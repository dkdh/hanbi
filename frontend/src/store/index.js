import Vue from "vue";
import Vuex from "vuex";
import { io } from "socket.io-client";
import Environment from "@/store/Environment.js"
import Robot from "@/store/Robot.js"
import Map from "@/store/Map.js"
import axios from "axios"

Vue.use(Vuex)

export default new Vuex.Store({
    namespaced: true,
    state: {
        category_idx: 0,
        socket: null,

        colors: {
            bg: "#65AC52",
            robot: "#D60707",
            obstacle: "#525252"
        },
        losts: [
            { name: "열쇠", img: 0, date: new Date() },
            { name: "지갑", img: 0, date: new Date() },
            { name: "갈비찜덮밥", img: 0, date: new Date() },
        ],
        map: {
            dsizeY: 500,
            dsizeX: 500
        },
        log: [
            { timestamp: String(new Date()), content: "vuex" },
            { timestamp: String(new Date()), content: "vuex2" },
            { timestamp: String(new Date()), content: "vuex3" }
        ],
        env: {
            temperature: 23,
            weather: "Cloud"
        },
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
        setSockets({ state, dispatch, commit }) {
            let { socket } = state
            //로직 1. 소켓 생성
            // //socket 등록
            socket = io("http://j5a102.p.ssafy.io:3000", {
                withCredentials: true,
            })

            //로직 2. 소켓 이벤트 정의
            socket.on("connect", async () => {
                console.log('connected')
                //로직 2-1. 연결되면 주기적으로 데이터를 받아옴
                setInterval(() => {
                    socket.emit("Map2Web")
                }, 300)

                setInterval(() => {
                    socket.emit("Robot2Web")
                }, 700)
                setInterval(() => {
                    socket.emit("History2Web")
                }, 1000)
                setInterval(() => {
                    socket.emit("Environment2Web")
                }, 1000)
                
                setInterval( () => {
                    socket.emit('request_stream_from_vue')
                }, 10)
                setInterval(() => {
                    if (state.url) {
                      socket.emit("uploadVideo", state.url)
                      state.url = null
                    }
                }, 1000)
            });

            socket.on('disconnect', function () {
                console.log('disconnected form` server_client.');
            });
            socket.on("connect_error", () => {
                // socket.auth.token = "abcd";
                setTimeout(() => {
                    try {
                        socket.connect();
                    } catch {
                        console.log("세션 연결에 실패했습니다")
                    }
                }, 1000);

            })
            socket.on("Map2Web", (data) => {
                if (data) {
                    console.log("Map : Get map data from server", data.length)
                    dispatch("Map/setMapping", data)
                } else {
                    console.log("Map : No Data from server")
                }
            })
            socket.on("Robot2Web", (data) => {
                if (data) {
                    // console.log("Robot : ", data)
                    dispatch("setRobot", data)
                } else {
                    console.log("Robot : No Data from server")
                }
            })
            socket.on("History2Web", (data) => {
                if (data) {
                    dispatch("setLog", data)
                } else {
                    console.log("Log : No Log from server")
                }
            })
            socket.on("Environment2Web", (data) => {
                if (data) {
                    // console.log("setEnvironment")
                    commit("Environment/setEnvironment", data)
                } else {
                    console.log("Environment : No Environment from werver")
                }
            })
            socket.on("stream_from_nodejs", (arrayBuffer) => {
                if (arrayBuffer) {
                    dispatch("setStreaming", arrayBuffer)
                }
            })
        },

        setRobot({ state, commit }, data) {
            state

            const { pos } = data
            const [x, y] = pos
            let robotDiv = document.querySelectorAll(".robot")
            // console.log("set Robot : ", state.map.dsizeY - y, x)
            for (let i = 0; i < robotDiv.length; i++) {
                robotDiv[i].style.top = (state.map.dsizeY - y) + "px";
                robotDiv[i].style.left = x + "px";
                robotDiv[i].style.backgroundColor = state.colors.robot
            }
            commit("Robot/setRobot", data)

        },
        setLog({ state }, data) {
            state, data
            state.log = data
            // console.log("log : ", data)
        },

        setStreaming({state}, arrayBuffer){
            state.streaming = arrayBuffer
        },

        stopRecord({state}, url){
            state.url = url
        }

    },
    getters: {},
    modules: {
        Environment,
        Robot,
        Map
    },
});
