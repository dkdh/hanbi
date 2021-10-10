export default {
    namespaced: true,
    state: {
        dSizeY: 500,
        dSizeX: 500,
        resol: 0.1,
        data: [],
        colors: {
            bg: "#5F5F5F",
            find_pre: "#F6",
            find_suf: "0C"
        }
    },
    mutations: {
        setEnvironment(state, data) {
            const { temperature, weather } = data
            state.temperature = temperature
            state.weather = weather
        },
        drawMapping(state) {
            console.log("drawMapping : ", state)
            
            const { dSizeY, dSizeX, data, colors } = state.Map

            if (!state.data.length) {
                for (let y = 0; y < dSizeY; y++) {
                    state.data.push([])
                    for (let x = 0; x < dSizeX; x++) {
                        state.data[y].push(colors.bg)
                    }
                }
            }

            let mapImg = document.querySelector(".mappingImg")
            if (!mapImg) return
            var ctx = mapImg.getContext('2d')

            //draw canvas
            for (let y = 0; y < dSizeY; y++) {
                for (let x = 0; x < dSizeX; x++) {
                    if (data[y][x]) {
                        // console.log("color : ", data[y][x])
                        ctx.fillStyle = data[y][x];
                        ctx.fillRect(x, dSizeY - y, 1, 1)

                    } else {
                        // console.log("drawMapping overflow ", y, x, data[y][x])
                    }
                }
            }
        }

    },

    actions: {
        setMapping({ state }, data) {
            try {
                if (!state.data.length) return

                const { colors, dSizeY } = state
                dSizeY

                // let mapImg = document.querySelector(".mappingImg")
                // var ctx = mapImg.getContext('2d')

                for (let i = 0; i < data.length; i++) {
                    // console.log("setMapping : ", data[i])
                    if (data[i].length != 3) continue
                    const [y, x, v] = data[i]
                    const next = colors.find_pre + v.toString(16) + colors.find_suf

                    state.data[y][x] = next
                }

                // state.data[50][state.a] = 0
                // state.a = state.a + 1
                // data
            } catch (e) {
                console.log("no mapImg", e)
            }
        },
        convertCoords({ state, rootState }, data) {
            //ROS에서 사용하는 좌표 정의
            const [x, y] = data
            const { dSizeX, dSizeY } = state
            const [rosSizeX, rosSizeY] = [350, 350]

            //변환
            const cvtX = (x - (dSizeX / 2)) / dSizeX * rosSizeX
            const cvtY = ((dSizeY / 2) - y) / dSizeY * rosSizeY

            console.log(cvtX, cvtY)

            console.log(rootState.losts)
            const { socket } = rootState
            if (socket == null) throw Error("No socket In convertCoords")

            socket.emit("Click2Server", { x: cvtX, y: cvtY })
        }
    }
}