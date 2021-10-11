import Vue from "vue";
import Vuex from "vuex";
import { io } from "socket.io-client";
import Environment from "@/store/Environment.js"
import Robot from "@/store/Robot.js"
import Map from "@/store/Map.js"
import Log from "@/store/Log.js"
import params from "@/js/config"
import axios from "axios"

Vue.use(Vuex)

export default new Vuex.Store({
    namespaced: true,
    state: {
        category_idx: 0,
        socket: null,
        logLength: 1,

        colors: {
            robot: "#5D6D7E",
            obstacle: "#525252",
            event: "#FFC300",
            emergency: "#FF5733"
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
                // setInterval(() => {
                //     state.socket.emit("Map2Web")
                // }, 300)

                setInterval(() => {
                    state.socket.emit("Robot2Web")
                }, 200)
                setInterval(() => {
                    state.socket.emit("History2Web")
                }, 1000)
                setInterval(() => {
                    state.socket.emit("Environment2Web")
                }, 500)

                setInterval(() => {
                    state.socket.emit('request_stream_from_vue')
                }, 10)
                setInterval(() => {
                    // console.log(111111111111111111)
                    if (state.url) {
                        // console.log(1111111111111111112222222222222222222)
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
                if (data.length) {
                    // console.log("Map : Get map data from server", data.length)
                    dispatch("Map/setMapping", data)
                } else {
                    console.log("Map : No Data from server")
                }
            })
            state.socket.on("Robot2Web", (data) => {
                if (data) {
                    // console.log("Robot : ", data)
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
        async setLog({ state }, data) {
            state.log = data
            if (data.length != state.logLength) {
                // console.log(state.logLength)
                // console.log(data.length)
                state.logLength = data.length
                // console.log(data[data.length - 1].content)
                // console.log(data[data.length - 1].timestamp)

                if (data[data.length - 1].content == "쓰레기 발견" || data[data.length - 1].content == "from nodejs") {
                    console.log("쓰레기 발견")
                } else {
                    let bytes = new Uint8Array(state.streaming);
                    let blob = new Blob([bytes], { type: "image/jpeg" });

                    const response = await axios({
                        method: 'GET',
                        url: "https://5q2pq5cazl.execute-api.ap-northeast-2.amazonaws.com/default/getImageUrl"
                    })
                    console.log('Response: ', response)
                    // Put request for upload to S3
                    const result = await fetch(response.data.uploadURL, {
                        method: 'PUT',
                        body: blob
                    })
                    console.log('Result: ', result)
                    let fileKey = response.data.Key;
                    let url = "https://iotiothanbi.s3.ap-northeast-2.amazonaws.com/" + fileKey;
                    console.log(url);
                    if (data[data.length - 1].content == "분실물 발견") {
                        state.socket.emit("uploadImage", url)
                    } else {
                        let arr = [url, data[data.length - 1].content]
                        state.socket.emit("uploadPicture", arr)
                    }
                }
            }
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
