import Vue from "vue";
import Vuex from "vuex";
import { io } from "socket.io-client";
import params from "../config"
// import fs from "fs"

Vue.use(Vuex)

export default new Vuex.Store({
    state: {
        category_idx: 0,
        socket: null,
        robot: {
            y: 0, x: 0
        },
        colors: {
            bg: "#65AC52",
            robot: "#D60707",
            obstacle: "#525252"
        },
        losts: [
            { name: "열쇠", img: 0, date: new Date() },
            { name: "지갑", img: 0, date: new Date() },
            { name: "갈비찜덮밥", img: 0, date: new Date() },
        ]
    },
    mutations: {
        setCategory(state, a) {
            // console.log("hi : ", a, state)
            state.category_idx = a
        },
        setRobot(state, pos) {
            state, pos
        }
    },
    actions: {
        setSockets({ state, dispatch }) {
            let { socket } = state
            //로직 1. 소켓 생성
            // //socket 등록
            socket = io(params["ip_server"], {
                withCredentials: true,
            })

            //로직 2. 소켓 이벤트 정의
            socket.on("connect", async () => {
                console.log('connected')
                //로직 2-1. 연결되면 주기적으로 데이터를 받아옴
                setInterval(() => {
                    socket.emit("Map2Web")
                }, 3000)

                setInterval(() => {
                    socket.emit("Robot2Web")
                }, 1500)
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
                    console.log("Map2Web : Get map data from server")
                    dispatch("setMap", data)
                } else {
                    console.log("Map2Web : No Data from server")
                }
            })
            socket.on("Robot2Web", (data) => {
                console.log(data)
                if (data) {
                    dispatch("setRobot", data)
                } else {
                    console.log("Set2Web : No Data from server")
                }
            })
        },
        setMap({ state }, data) {
            try {
                let { map, colors } = state
                map, data
                const dSizeX = data.length
                const dSizeY = data[0].length

                let mapImg = document.querySelector("#mapImg")
                var ctx = mapImg.getContext('2d')

                ctx.clearRect(0, 0, dSizeX, dSizeY)

                for (let y = 0; y < dSizeY; y++) {
                    for (let x = 0; x < dSizeX; x++) {
                        //! 색 수정, 알고리즘 수정
                        if (data[y][x] > 50) ctx.fillStyle = colors.bg;
                        if (data[y][x] <= 50) ctx.fillStyle = colors.obstacle;
                        ctx.fillRect(x, y, 1, 1)
                    }
                }
                console.log(" setMap : ", mapImg)
            } catch {
                console.log("no mapImg")
            }
        },
        setRobot({ state }, data) {
            try {
                state
                const [y, x] = data
                let robotDiv = document.querySelector("#robot")
                console.log(y, x)
                robotDiv.style.bottom = y + "px";
                robotDiv.style.left = x + "px";
                robotDiv.style.backgroundColor = state.colors.robot
            } catch {
                console.log("no Robot")
            }
        }
    },
    getters: {},
    modules: {
    },
});
