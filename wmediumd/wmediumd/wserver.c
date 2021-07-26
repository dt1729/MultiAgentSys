/*
 *	wmediumd_server - server for on-the-fly modifications for wmediumd
 *	Copyright (c) 2016, Patrick Grosse <patrick.grosse@uni-muenster.de>
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version 2
 *	of the License, or (at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 *	02110-1301, USA.
 */

#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <errno.h>
#include <event.h>

#include "wserver.h"
#include "wmediumd_dynamic.h"
#include "wserver_messages.h"


#define LOG_PREFIX "W_SRV: "

/**
 * Global listen socket
 */
static int listen_soc;

/**
 * Global thread
 */
static pthread_t server_thread;

/**
 * Event base for server events
 */
static struct event_base *server_event_base;

/**
 * Stores the old sig handler for SIGINT
 */
static __sighandler_t old_sig_handler;

/**
 * Handle the SIGINT signal
 * @param param The param passed to by signal()
 */
void handle_sigint(int param) {
    UNUSED(param);
    stop_wserver();
    exit(EXIT_SUCCESS);
}

/* Existing link is from from -> to; copy to other dir */
static void mirror_link_(struct request_ctx *ctx, int from, int to, int signal)
{
	ctx->ctx->snr_matrix[ctx->ctx->num_stas * to + from] = signal;
	ctx->ctx->snr_matrix[ctx->ctx->num_stas * from + to] = signal;
}


static void calc_signal(struct request_ctx *ctx)
{
	int txpower, path_loss, gains, signal, from, to;

	for (from = 0; from < ctx->ctx->num_stas; from++) {
		for (to = 0; to < ctx->ctx->num_stas; to++) {
			if (from == to)
				continue;
			txpower = ctx->ctx->sta_array[from]->tx_power;
			if (ctx->ctx->sta_array[to]->isap == 1)
				txpower = ctx->ctx->sta_array[to]->tx_power;
			path_loss = ctx->ctx->calc_path_loss(ctx->ctx->path_loss_param,
					ctx->ctx->sta_array[to], ctx->ctx->sta_array[from]);
			gains = (txpower + ctx->ctx->sta_array[from]->gain + ctx->ctx->sta_array[to]->gain);
			signal = gains - path_loss - ctx->ctx->noise_threshold;
			mirror_link_(ctx, from, to, signal);
		}
	}
}

/**
 * Create the listening socket
 * @param ctx The wmediumd context
 * @return The FD of the socket
 */
int create_listen_socket(struct wmediumd *ctx) {
    int soc = socket(AF_UNIX, SOCK_STREAM, 0);
    if (soc < 0) {
        w_logf(ctx, LOG_ERR, LOG_PREFIX "Socket not created: %s\n", strerror(errno));
        return -1;
    }
    w_logf(ctx, LOG_DEBUG, LOG_PREFIX "Socket created\n");

    int unlnk_ret = unlink(WSERVER_SOCKET_PATH);
    if (unlnk_ret != 0 && errno != ENOENT) {
        w_logf(ctx, LOG_ERR, LOG_PREFIX "Cannot remove old UNIX socket at '\" WSERVER_SOCKET_PATH \"': %s\n",
               strerror(errno));
        close(soc);
        return -1;
    }
    struct sockaddr_un saddr = {AF_UNIX, WSERVER_SOCKET_PATH};
    int retval = bind(soc, (struct sockaddr *) &saddr, sizeof(saddr));
    if (retval < 0) {
        w_logf(ctx, LOG_ERR, LOG_PREFIX "Bind failed: %s\n", strerror(errno));
        close(soc);
        return -1;
    }
    w_logf(ctx, LOG_DEBUG, LOG_PREFIX "Bound to UNIX socket '" WSERVER_SOCKET_PATH "'\n");

    retval = listen(soc, 10);
    if (retval < 0) {
        w_logf(ctx, LOG_ERR, LOG_PREFIX "Listen failed: %s\n", strerror(errno));
        close(soc);
        return -1;
    }

    return soc;
}

/**
 * Accept incoming connections
 * @param listen_soc The FD of the server socket
 * @return The FD of the client socket
 */
int accept_connection(int listen_soc) {
    struct sockaddr_in claddr;
    socklen_t claddr_size = sizeof(claddr);

    int soc = accept(listen_soc, (struct sockaddr *) &claddr, &claddr_size);
    if (soc < 0) {
        return -1;
    }
    return soc;
}

int handle_snr_update_request(struct request_ctx *ctx, const snr_update_request *request) {
    snr_update_response response;
    response.request = *request;

    if (ctx->ctx->snr_matrix != NULL) {
    	struct station *sender = NULL;
    	struct station *receiver = NULL;
    	struct station *station;

        pthread_rwlock_wrlock(&snr_lock);

        list_for_each_entry(station, &ctx->ctx->stations, list) {
            if (memcmp(&request->from_addr, station->addr, ETH_ALEN) == 0) {
                sender = station;
            }
            if (memcmp(&request->to_addr, station->addr, ETH_ALEN) == 0) {
                receiver = station;
            }
        }

        if (!sender || !receiver) {
            w_logf(ctx->ctx, LOG_WARNING,
                   LOG_PREFIX "Could not perform SNR update from=" MAC_FMT ", to=" MAC_FMT ", snr=%d; station(s) not found\n",
                   MAC_ARGS(request->from_addr), MAC_ARGS(request->to_addr), request->snr);
            response.update_result = WUPDATE_INTF_NOTFOUND;
        } else {
            w_logf(ctx->ctx, LOG_NOTICE, LOG_PREFIX "Performing SNR update: from=" MAC_FMT ", to=" MAC_FMT ", snr=%d\n",
                   MAC_ARGS(sender->addr), MAC_ARGS(receiver->addr), request->snr);

            mirror_link_(ctx, sender->index, receiver->index, request->snr);
            response.update_result = WUPDATE_SUCCESS;
        }
        pthread_rwlock_unlock(&snr_lock);
    } else {
        response.update_result = WUPDATE_WRONG_MODE;
    }
    int ret = wserver_send_msg(ctx->sock_fd, &response, snr_update_response);
    if (ret < 0) {
        w_logf(ctx->ctx, LOG_ERR, "Error on SNR update response: %s\n", strerror(abs(ret)));
        return WACTION_ERROR;
    }
    return ret;
}

int handle_position_update_request(struct request_ctx *ctx, const position_update_request *request) {
    position_update_response response;
    response.request = *request;

    if (ctx->ctx->error_prob_matrix == NULL) {
    	struct station *sender = NULL;
    	struct station *station;

        pthread_rwlock_wrlock(&snr_lock);

        list_for_each_entry(station, &ctx->ctx->stations, list) {
			if (memcmp(&request->sta_addr, station->addr, ETH_ALEN) == 0) {
				sender = station;
				sender->x = request->posX;
				sender->y = request->posY;
				sender->z = request->posZ;
			}
        }

        w_logf(ctx->ctx, LOG_NOTICE, LOG_PREFIX "Performing Position update: for=" MAC_FMT ", position=%f,%f,%f\n",
			   MAC_ARGS(request->sta_addr), request->posX, request->posY, request->posZ);

		calc_signal(ctx);
		response.update_result = WUPDATE_SUCCESS;

        pthread_rwlock_unlock(&snr_lock);
    } else {
        response.update_result = WUPDATE_WRONG_MODE;
    }
    int ret = wserver_send_msg(ctx->sock_fd, &response, position_update_response);
    return ret;
}

int handle_txpower_update_request(struct request_ctx *ctx, const txpower_update_request *request) {
    txpower_update_response response;
    response.request = *request;

    if (ctx->ctx->error_prob_matrix == NULL) {
    	struct station *sender = NULL;
    	struct station *station;

        pthread_rwlock_wrlock(&snr_lock);

        list_for_each_entry(station, &ctx->ctx->stations, list) {
			if (memcmp(&request->sta_addr, station->addr, ETH_ALEN) == 0) {
				sender = station;
				sender->tx_power = request->txpower_;
			}
        }

		w_logf(ctx->ctx, LOG_NOTICE, LOG_PREFIX "Performing TxPower update: for=" MAC_FMT ", txpower=%d\n",
			   MAC_ARGS(request->sta_addr), request->txpower_);

		calc_signal(ctx);
		response.update_result = WUPDATE_SUCCESS;

        pthread_rwlock_unlock(&snr_lock);
    } else {
        response.update_result = WUPDATE_WRONG_MODE;
    }
    int ret = wserver_send_msg(ctx->sock_fd, &response, txpower_update_response);
    return ret;
}

int handle_gaussian_random_update_request(struct request_ctx *ctx, const gaussian_random_update_request *request) {
	gaussian_random_update_response response;
    response.request = *request;

    if (ctx->ctx->error_prob_matrix == NULL) {
    	struct station *sender = NULL;
    	struct station *station;

        pthread_rwlock_wrlock(&snr_lock);

        list_for_each_entry(station, &ctx->ctx->stations, list) {
			if (memcmp(&request->sta_addr, station->addr, ETH_ALEN) == 0) {
				sender = station;
				sender->gRandom = request->gaussian_random_;
			}
        }

		w_logf(ctx->ctx, LOG_NOTICE, LOG_PREFIX "Performing Gaussian Random update: for=" MAC_FMT ", gRandom=%d\n",
			   MAC_ARGS(request->sta_addr), request->gaussian_random_);

		calc_signal(ctx);
		response.update_result = WUPDATE_SUCCESS;

        pthread_rwlock_unlock(&snr_lock);
    } else {
        response.update_result = WUPDATE_WRONG_MODE;
    }
    int ret = wserver_send_msg(ctx->sock_fd, &response, gaussian_random_update_response);
    return ret;
}

int handle_gain_update_request(struct request_ctx *ctx, const gain_update_request *request) {
	gain_update_response response;
    response.request = *request;

    if (ctx->ctx->error_prob_matrix == NULL) {
    	struct station *sender = NULL;
    	struct station *station;

        pthread_rwlock_wrlock(&snr_lock);

        list_for_each_entry(station, &ctx->ctx->stations, list) {
			if (memcmp(&request->sta_addr, station->addr, ETH_ALEN) == 0) {
				sender = station;
				sender->gain = request->gain_;
			}
        }

        w_logf(ctx->ctx, LOG_NOTICE, LOG_PREFIX "Performing Gain update: for=" MAC_FMT ", gain=%d\n",
			   MAC_ARGS(request->sta_addr), request->gain_);

        calc_signal(ctx);
		response.update_result = WUPDATE_SUCCESS;

        pthread_rwlock_unlock(&snr_lock);
    } else {
        response.update_result = WUPDATE_WRONG_MODE;
    }
    int ret = wserver_send_msg(ctx->sock_fd, &response, gain_update_response);
    return ret;
}


int handle_errprob_update_request(struct request_ctx *ctx, const errprob_update_request *request) {
    errprob_update_response response;
    response.request = *request;

    if (ctx->ctx->error_prob_matrix != NULL) {
        struct station *sender = NULL;
        struct station *receiver = NULL;
        struct station *station;

        pthread_rwlock_wrlock(&snr_lock);

        list_for_each_entry(station, &ctx->ctx->stations, list) {
            if (memcmp(&request->from_addr, station->addr, ETH_ALEN) == 0) {
                sender = station;
            }
            if (memcmp(&request->to_addr, station->addr, ETH_ALEN) == 0) {
                receiver = station;
            }
        }

        double errprob = custom_fixed_point_to_floating_point(request->errprob);

        if (!sender || !receiver) {
            w_logf(ctx->ctx, LOG_WARNING,
                   LOG_PREFIX "Could not perform ERRPROB update from=" MAC_FMT ", to=" MAC_FMT ", errprob=%f; station(s) not found\n",
                   MAC_ARGS(request->from_addr), MAC_ARGS(request->to_addr), errprob);
            response.update_result = WUPDATE_INTF_NOTFOUND;
        } else {
            w_logf(ctx->ctx, LOG_NOTICE,
                   LOG_PREFIX "Performing ERRPROB update: from=" MAC_FMT ", to=" MAC_FMT ", errprob=%f\n",
                   MAC_ARGS(sender->addr), MAC_ARGS(receiver->addr), errprob);
            ctx->ctx->error_prob_matrix[sender->index * ctx->ctx->num_stas + receiver->index] = errprob;
            ctx->ctx->error_prob_matrix[receiver->index * ctx->ctx->num_stas + sender->index] = errprob;
            response.update_result = WUPDATE_SUCCESS;
        }
        pthread_rwlock_unlock(&snr_lock);
    } else {
        response.update_result = WUPDATE_WRONG_MODE;
    }
    int ret = wserver_send_msg(ctx->sock_fd, &response, errprob_update_response);
    if (ret < 0) {
        w_logf(ctx->ctx, LOG_ERR, "Error on ERRPROB update response: %s\n", strerror(abs(ret)));
        return WACTION_ERROR;
    }
    return ret;
}

int handle_specprob_update_request(struct request_ctx *ctx, const specprob_update_request *request) {
    specprob_update_response response;
    memcpy(response.from_addr, request->from_addr, ETH_ALEN);
    memcpy(response.to_addr, request->to_addr, ETH_ALEN);

    if (ctx->ctx->station_err_matrix != NULL) {
        struct station *sender = NULL;
        struct station *receiver = NULL;
        struct station *station;

        pthread_rwlock_wrlock(&snr_lock);

        list_for_each_entry(station, &ctx->ctx->stations, list) {
            if (memcmp(&request->from_addr, station->addr, ETH_ALEN) == 0) {
                sender = station;
            }
            if (memcmp(&request->to_addr, station->addr, ETH_ALEN) == 0) {
                receiver = station;
            }
        }

        if (!sender || !receiver) {
            w_logf(ctx->ctx, LOG_WARNING,
                   LOG_PREFIX "Could not perform SPECPROB update from=" MAC_FMT ", to=" MAC_FMT "; station(s) not found\n",
                   MAC_ARGS(request->from_addr), MAC_ARGS(request->to_addr));
            response.update_result = WUPDATE_INTF_NOTFOUND;
        } else {
            w_logf(ctx->ctx, LOG_NOTICE,
                   LOG_PREFIX "Performing SPECPROB update: from=" MAC_FMT ", to=" MAC_FMT "\n",
                   MAC_ARGS(sender->addr), MAC_ARGS(receiver->addr));
            double *specific_mat = malloc(sizeof(double) * SPECIFIC_MATRIX_MAX_SIZE_IDX * SPECIFIC_MATRIX_MAX_RATE_IDX);
            if (!specific_mat) {
                w_logf(ctx->ctx, LOG_ERR, "Error during allocation of memory in handle_specprob_update_request wmediumd/wserver.c\n");
                // should be different type of error here
                response.update_result = WUPDATE_WRONG_MODE;
                goto out;
            }
            for (int i = 0; i < SPECIFIC_MATRIX_MAX_SIZE_IDX * SPECIFIC_MATRIX_MAX_RATE_IDX; i++) {
                specific_mat[i] = custom_fixed_point_to_floating_point(request->errprob[i]);
            }
            if (ctx->ctx->station_err_matrix[sender->index * ctx->ctx->num_stas + receiver->index] != NULL) {
                free(ctx->ctx->station_err_matrix[sender->index * ctx->ctx->num_stas + receiver->index]);
            }
            ctx->ctx->station_err_matrix[sender->index * ctx->ctx->num_stas + receiver->index] = specific_mat;
            response.update_result = WUPDATE_SUCCESS;
        }
out:
        pthread_rwlock_unlock(&snr_lock);
    } else {
        response.update_result = WUPDATE_WRONG_MODE;
    }
    int ret = wserver_send_msg(ctx->sock_fd, &response, specprob_update_response);
    if (ret < 0) {
        w_logf(ctx->ctx, LOG_ERR, "Error on SPECPROB update response: %s\n", strerror(abs(ret)));
        return WACTION_ERROR;
    }
    return ret;
}

int handle_delete_by_id_request(struct request_ctx *ctx, station_del_by_id_request *request) {
    station_del_by_id_response response;
    response.request = *request;
    int ret = del_station_by_id(ctx->ctx, request->id);
    if (ret) {
        if (ret == -ENODEV) {
            w_logf(ctx->ctx, LOG_WARNING, LOG_PREFIX
                    "Station with ID %d could not be found\n", request->id);
            response.update_result = WUPDATE_INTF_NOTFOUND;
        } else {
            w_logf(ctx->ctx, LOG_ERR, "Error on delete by id request: %s\n", strerror(abs(ret)));
            return WACTION_ERROR;
        }
    } else {
        w_logf(ctx->ctx, LOG_NOTICE, LOG_PREFIX
                "Station with ID %d successfully deleted\n", request->id);
        response.update_result = WUPDATE_SUCCESS;
    }
    ret = wserver_send_msg(ctx->sock_fd, &response, station_del_by_id_response);
    if (ret < 0) {
        w_logf(ctx->ctx, LOG_ERR, "Error on delete by id response: %s\n", strerror(abs(ret)));
        return WACTION_ERROR;
    }
    return ret;
}

int handle_delete_by_mac_request(struct request_ctx *ctx, station_del_by_mac_request *request) {
    station_del_by_mac_response response;
    response.request = *request;
    int ret = del_station_by_mac(ctx->ctx, request->addr);
    if (ret) {
        if (ret == -ENODEV) {
            w_logf(ctx->ctx, LOG_WARNING, LOG_PREFIX
                    "Station with MAC " MAC_FMT " could not be found\n", MAC_ARGS(request->addr));
            response.update_result = WUPDATE_INTF_NOTFOUND;
        } else {
            w_logf(ctx->ctx, LOG_ERR, "Error %d on delete by mac request: %s\n", ret, strerror(abs(ret)));
            return WACTION_ERROR;
        }
    } else {
        w_logf(ctx->ctx, LOG_NOTICE, LOG_PREFIX
                "Station with MAC " MAC_FMT " successfully deleted\n", MAC_ARGS(request->addr));
        response.update_result = WUPDATE_SUCCESS;
    }
    ret = wserver_send_msg(ctx->sock_fd, &response, station_del_by_mac_response);
    if (ret < 0) {
        w_logf(ctx->ctx, LOG_ERR, "Error on delete by mac response: %s\n", strerror(abs(ret)));
        return WACTION_ERROR;
    }
    return ret;
}

int handle_add_request(struct request_ctx *ctx, station_add_request *request) {
    int ret = add_station(ctx->ctx, request->addr);
    station_add_response response;
    response.request = *request;
    if (ret < 0) {
        if (ret == -EEXIST) {
            w_logf(ctx->ctx, LOG_WARNING, LOG_PREFIX
                    "Station with MAC " MAC_FMT " already exists\n", MAC_ARGS(request->addr));
            response.created_id = 0;
            response.update_result = WUPDATE_INTF_DUPLICATE;
        } else {
            w_logf(ctx->ctx, LOG_ERR, "Error on add request: %s\n", strerror(abs(ret)));
            return WACTION_ERROR;
        }
    } else {
        w_logf(ctx->ctx, LOG_NOTICE, LOG_PREFIX
                "Added station with MAC " MAC_FMT " and ID %d\n", MAC_ARGS(request->addr), ret);
        response.created_id = ret;
        response.update_result = WUPDATE_SUCCESS;
    }
    ret = wserver_send_msg(ctx->sock_fd, &response, station_add_response);
    if (ret < 0) {
        w_logf(ctx->ctx, LOG_ERR, "Error on add response: %s\n", strerror(abs(ret)));
        return WACTION_ERROR;
    }
    return ret;
}

int parse_recv_msg_rest_error(struct wmediumd *ctx, int value) {
    if (value > 0) {
        return value;
    } else {
        w_logf(ctx, LOG_ERR, "Error on receive msg rest: %s\n", strerror(abs(value)));
        return WACTION_ERROR;
    }
}

int receive_handle_request(struct request_ctx *ctx) {
    wserver_msg base;
    int recv_type;
    int ret = wserver_recv_msg_base(ctx->sock_fd, &base, &recv_type);
    if (ret > 0) {
        return ret;
    } else if (ret < 0) {
        w_logf(ctx->ctx, LOG_ERR, "Error on receive base request: %s\n", strerror(abs(ret)));
        return WACTION_ERROR;
    }
    if (recv_type == WSERVER_SHUTDOWN_REQUEST_TYPE) {
        return WACTION_CLOSE;
    } else if (recv_type == WSERVER_SNR_UPDATE_REQUEST_TYPE) {
        snr_update_request request;
        if ((ret = wserver_recv_msg(ctx->sock_fd, &request, snr_update_request))) {
            return parse_recv_msg_rest_error(ctx->ctx, ret);
        } else {
            return handle_snr_update_request(ctx, &request);
        }
    } else if (recv_type == WSERVER_ERRPROB_UPDATE_REQUEST_TYPE) {
        errprob_update_request request;
        if ((ret = wserver_recv_msg(ctx->sock_fd, &request, errprob_update_request))) {
            return parse_recv_msg_rest_error(ctx->ctx, ret);
        } else {
            return handle_errprob_update_request(ctx, &request);
        }
    } else if (recv_type == WSERVER_SPECPROB_UPDATE_REQUEST_TYPE) {
        specprob_update_request request;
        if ((ret = wserver_recv_msg(ctx->sock_fd, &request, specprob_update_request))) {
            return parse_recv_msg_rest_error(ctx->ctx, ret);
        } else {
            return handle_specprob_update_request(ctx, &request);
        }
    } else if (recv_type == WSERVER_DEL_BY_MAC_REQUEST_TYPE) {
        station_del_by_mac_request request;
        if ((ret = wserver_recv_msg(ctx->sock_fd, &request, station_del_by_mac_request))) {
            return parse_recv_msg_rest_error(ctx->ctx, ret);
        } else {
            return handle_delete_by_mac_request(ctx, &request);
        }
    } else if (recv_type == WSERVER_DEL_BY_ID_REQUEST_TYPE) {
        station_del_by_id_request request;
        if ((ret = wserver_recv_msg(ctx->sock_fd, &request, station_del_by_id_request))) {
            return parse_recv_msg_rest_error(ctx->ctx, ret);
        } else {
            return handle_delete_by_id_request(ctx, &request);
        }
    } else if (recv_type == WSERVER_ADD_REQUEST_TYPE) {
        station_add_request request;
        if ((ret = wserver_recv_msg(ctx->sock_fd, &request, station_add_request))) {
            return parse_recv_msg_rest_error(ctx->ctx, ret);
        } else {
            return handle_add_request(ctx, &request);
        }
    } else if (recv_type == WSERVER_POSITION_UPDATE_REQUEST_TYPE) {
        position_update_request request;
        if ((ret = wserver_recv_msg(ctx->sock_fd, &request, position_update_request))) {
            return parse_recv_msg_rest_error(ctx->ctx, ret);
        } else {
            return handle_position_update_request(ctx, &request);
        }
    } else if (recv_type == WSERVER_TXPOWER_UPDATE_REQUEST_TYPE) {
		txpower_update_request request;
		if ((ret = wserver_recv_msg(ctx->sock_fd, &request, txpower_update_request))) {
			return parse_recv_msg_rest_error(ctx->ctx, ret);
		} else {
			return handle_txpower_update_request(ctx, &request);
		}
    } else if (recv_type == WSERVER_GAIN_UPDATE_REQUEST_TYPE) {
		gain_update_request request;
		if ((ret = wserver_recv_msg(ctx->sock_fd, &request, gain_update_request))) {
			return parse_recv_msg_rest_error(ctx->ctx, ret);
		} else {
			return handle_gain_update_request(ctx, &request);
		}
    } else if (recv_type == WSERVER_GAUSSIAN_RANDOM_UPDATE_REQUEST_TYPE) {
		gaussian_random_update_request request;
		if ((ret = wserver_recv_msg(ctx->sock_fd, &request, gaussian_random_update_request))) {
			return parse_recv_msg_rest_error(ctx->ctx, ret);
		} else {
			return handle_gaussian_random_update_request(ctx, &request);
		}
    } else {
        return -1;
    }
}

struct accept_context {
    struct wmediumd *wctx;
    int server_socket;
    int client_socket;
    pthread_t *thread;
};

void *handle_accepted_connection(void *d_ptr) {
    struct accept_context *actx = d_ptr;
    struct request_ctx rctx;
    rctx.ctx = actx->wctx;
    rctx.sock_fd = actx->client_socket;
    w_logf(rctx.ctx, LOG_INFO, LOG_PREFIX "Client connected\n");
    while (1) {
        int action_resp;
        w_logf(rctx.ctx, LOG_INFO, LOG_PREFIX "Waiting for request...\n");
        action_resp = receive_handle_request(&rctx);
        if (action_resp == WACTION_DISCONNECTED) {
            w_logf(rctx.ctx, LOG_INFO, LOG_PREFIX "Client has disconnected\n");
            break;
        } else if (action_resp == WACTION_ERROR) {
            w_logf(rctx.ctx, LOG_INFO, LOG_PREFIX "Disconnecting client because of error\n");
            break;
        } else if (action_resp == WACTION_CLOSE) {
            w_logf(rctx.ctx, LOG_INFO, LOG_PREFIX "Closing server\n");
            event_base_loopbreak(server_event_base);
            break;
        }
    }
    close(rctx.sock_fd);
    free(actx);
    return NULL;
}

void on_listen_event(int fd, short what, void *wctx) {
    UNUSED(fd);
    UNUSED(what);
    struct accept_context *actx = malloc(sizeof(struct accept_context));
    if (!actx) {
        w_logf(wctx, LOG_ERR, "Error during allocation of memory in on_listen_event wmediumd/wserver.c\n");
        return;
    }
    actx->wctx = wctx;
    actx->server_socket = fd;
    actx->thread = malloc(sizeof(pthread_t));
    if (!actx->thread) {
        w_logf(wctx, LOG_ERR, "Error during allocation of memory in on_listen_event wmediumd/wserver.c\n");
        free(actx);
        return;
    }
    actx->client_socket = accept_connection(actx->server_socket);
    if (actx->client_socket < 0) {
        w_logf(actx->wctx, LOG_ERR, LOG_PREFIX "Accept failed: %s\n", strerror(errno));
    } else {
        pthread_create(actx->thread, NULL, handle_accepted_connection, actx);
    }
}

/**
 * Run the server using the given wmediumd context
 * @param ctx The wmediumd context
 * @return NULL, required for pthread
 */
void *run_wserver(void *ctx) {
    struct event *accept_event;

    old_sig_handler = signal(SIGINT, handle_sigint);

    listen_soc = create_listen_socket(ctx);
    if (listen_soc < 0) {
        return NULL;
    }
    w_logf(ctx, LOG_DEBUG, LOG_PREFIX "Listening for incoming connection\n");

    evutil_make_socket_nonblocking(listen_soc);
    server_event_base = event_base_new();
    accept_event = event_new(server_event_base, listen_soc, EV_READ | EV_PERSIST, on_listen_event, ctx);
    event_add(accept_event, NULL);

    w_logf(ctx, LOG_DEBUG, LOG_PREFIX "Waiting for client to connect...\n");
    event_base_dispatch(server_event_base);

    event_free(accept_event);
    event_base_free(server_event_base);
    stop_wserver();
    return NULL;
}

int start_wserver(struct wmediumd *ctx) {
    return pthread_create(&server_thread, NULL, run_wserver, ctx);
}

void stop_wserver() {
    signal(SIGINT, old_sig_handler);
    pthread_cancel(server_thread);
    pthread_detach(server_thread);
    printf("\n" LOG_PREFIX "shutting down wserver\n");
    close(listen_soc);
    unlink(WSERVER_SOCKET_PATH);
}
