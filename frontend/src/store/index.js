import Vue from "vue";
import Vuex from "vuex";
import { io } from "socket.io-client";
import Environment from "@/store/Environment.js"
import Robot from "@/store/Robot.js"
import Map from "@/store/Map.js"
import Log from "@/store/Log.js"
import params from "@/js/config"
Vue.use(Vuex)

export default new Vuex.Store({
    namespaced: true,
    state: {
        category_idx: 0,
        socket: null,

        colors: {
            bg: "#65AC52",
            robot: "#D60707",
            obstacle: "#525252",
            event: "#FDEED7",
            emergency: "FDEED7"
        },
        losts: [
            { name: "열쇠", img: 0, date: new Date() },
            { name: "지갑", img: 0, date: new Date() },
            { name: "갈비찜덮밥", img: 0, date: new Date() },
        ],
        // map: {
        //     dsizeY: 500,
        //     dsizeX: 500
        // },
        a: 0,
        streaming: null,
        url: null,
    },
    mutations: {
        setCategory(state, a) {
            // console.log("hi : ", a, state)S
            state.category_idx = a
        },
        setRobot(state, pos) {
            state, pos
        },
        setLosts(state, lost) {
            // console.log(lost)
            state.losts = lost
        },
    },
    actions: {
        setSockets({ state, dispatch }) {
            // let { socket } = state
            //로직 1. 소켓 생성
            // //socket 등록
            state.socket = io(params.host, {
                withCredentials: true,
            })

            //로직 2. 소켓 이벤트 정의
            state.socket.on("connect", async () => {
                console.log('connected')
                //로직 2-1. 연결되면 주기적으로 데이터를 받아옴
                setInterval(() => {
                    state.socket.emit("Map2Web")
                }, 300)

                setInterval(() => {
                    state.socket.emit("Robot2Web")
                }, 700)
                setInterval(() => {
                    state.socket.emit("History2Web")
                }, 1000)
                setInterval(() => {
                    state.socket.emit("Environment2Web")
                }, 1000)

                setInterval(() => {
                    state.socket.emit('request_stream_from_vue')
                }, 10)
                setInterval(() => {
                    if (state.url) {
                        state.socket.emit("uploadVideo", state.url)
                        state.url = null
                    }
                }, 1000)
            });

            state.socket.on('disconnect', function () {
                console.log('disconnected form` server_client.');
            });
            state.socket.on("connect_error", () => {
                // state.socket.auth.token = "abcd";
                setTimeout(() => {
                    try {
                        state.socket.connect();
                    } catch {
                        console.log("세션 연결에 실패했습니다")
                    }
                }, 1000);

            })
            state.socket.on("Map2Web", (data) => {
                if (data) {
                    console.log("Map : Get map data from server", data.length)
                    dispatch("Map/setMapping", data)
                } else {
                    console.log("Map : No Data from server")
                }
            })
            state.socket.on("Robot2Web", (data) => {
                if (data) {
                    console.log("Robot : ", data)
                    dispatch("Robot/setRobot", data)
                } else {
                    console.log("Robot : No Data from server")
                }
            })
            state.socket.on("History2Web", (data) => {
                if (data) {
                    dispatch("Log/setLog", data)
                } else {
                    console.log("Log : No Log from server")
                }
            })
            state.socket.on("Environment2Web", (data) => {
                if (data) {
                    // console.log("setEnvironment")
                    dispatch("Environment/setEnvironment", data)
                } else {
                    console.log("Environment : No Environment from werver")
                }
            })
            state.socket.on("stream_from_nodejs", (arrayBuffer) => {
                if (arrayBuffer) {
                    dispatch("setStreaming", arrayBuffer)
                }
            })
        },

        setStreaming({ state }, arrayBuffer) {
            state.streaming = arrayBuffer
        },

        stopRecord({ state }, url) {
            state.url = url
        },
    },
    getters: {},
    modules: {
        Environment,
        Robot,
        Map,
        Log
    },
});
