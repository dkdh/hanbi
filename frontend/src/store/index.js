import Vue from "vue";
import Vuex from "vuex";
import { io } from "socket.io-client";
import params from "../config"
Vue.use(Vuex)

export default new Vuex.Store({
    state: {
        category_idx: 0,
        socket: null
    },
    mutations: {
        setCategory(state, a) {
            // console.log("hi : ", a, state)
            state.category_idx = a
        }
    },
    actions: {
        setSockets({ state }) {
            let { socket } = state

            // //socket 등록
            socket = io(params["ip_server"] || 'https://localhost:8080')
            socket.on("connect", () => {
                console.log('connected')
            });

            socket.on('disconnect', function () {
                console.log('disconnected form` server_client.');
            });

            socket.on("connect_error", () => {
                // socket.auth.token = "abcd";
                try {
                    setTimeout(() => {
                        socket.connect();
                    }, 1000);
                } catch {
                    console.log("세션 연결에 실패했습니다")
                }
            })
        }
    },
    getters: {},
    modules: {
    },
});
