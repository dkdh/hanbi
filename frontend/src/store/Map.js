export default {
    namespaced: true,
    state: {
        dSizeY: 500,
        dSizeX: 500,
        dSize: 500 * 500,
        resol: 0.1,
        data: [],
        colors: {
            bg: "#333333",
            find_pre: "#F6",
            find_suf: "0C"
        },
        percentage: 0,
        isVisited: [],
    },
    mutations: {
        drawMapping(state) {
            console.log("drawMapping : ")
            //mappingImg에 맵을 그리는 뮤테이션

            const { dSizeY, dSizeX, data, colors } = state

            if (!state.data.length) {
                //맵 데이터가 비어있을 경우
                for (let y = 0; y < dSizeY; y++) {
                    state.data.push([])
                    state.isVisited.push([])
                    for (let x = 0; x < dSizeX; x++) {
                        state.data[y].push(colors.bg)
                        state.isVisited[y].push(false)
                    }
                }
            }

            // 랜더링할 맵 객체 지정
            let mapImg = document.querySelector(".mappingImg")
            if (!mapImg) return
            var ctx = mapImg.getContext('2d')

            // 랜더링
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
            // 레이더로 탐지한 결과를 state.data에 갱신
            try {
                if (!state.data.length) return

                const { dSizeY } = state
                dSizeY

                for (let i = 0; i < data.length; i++) {
                    if (data[i].length != 3) continue
                    const [y, x, v] = data[i]

                    const value = Math.floor(v * 255).toString(16)
                    // const next = state.colors.find_pre + value + state.colors.find_suf
                    const next = "#" + value + value + value
                    // console.log(next, data[i])
                    if (!state.isVisited[y][x]) {
                        state.isVisited[y][x] = true
                        state.percentage += 1

                    }
                    state.data[y][x] = next
                }

            } catch (e) {
                console.log("no mapImg", e)
            }
        },
        click({ state, rootState }, data) {
            // Map 클릭시 좌표를 전송하는 액션


            const [x, y] = data
            const { dSizeX, dSizeY, resol } = state
            // const [rosSizeX, rosSizeY] = [350, 350]

            //변환
            const cvtX = (x * resol - (dSizeX * resol / 2))
            const cvtY = ((dSizeY * resol / 2) - y * resol)

            console.log(x, y, cvtX, cvtY)

            console.log(rootState.losts)
            const { socket } = rootState
            if (socket == null) throw Error("No socket In convertCoords")

            socket.emit("Click2Server", { x: cvtX, y: cvtY })
        },
        renderLog({ state, rootState }, data) {
            //map에 log 기록을 남기는 함수
            console.log("renderLog")
            state, data, rootState
            const { dSizeY, dSizeX, resol } = state
            const { log } = rootState.Log

            let logDOM = document.querySelectorAll('.event');
            console.log("renderLog:logDom", logDOM)
            // 이벤트 개수 세기, 객체 설정
            for (let i = 0; i < logDOM.length; i++) {
                // console.log("log : ", log[i])
                const pose = log[i].pose
                const x = pose[0]
                const y = pose[1]

                logDOM[i].style.top = (y / resol  + dSizeY / 2) + "px"
                logDOM[i].style.left = (x / resol + dSizeX / 2)  + "px"
                if (log[i].emergency === 1) {
                    logDOM[i].classList.add("emergency")
                    logDOM[i].style.backgroundColor = rootState.colors["emergency"]
                } else {
                    logDOM[i].style.backgroundColor = rootState.colors["event"]
                    logDOM[i].style.icon = "el-icon-warning-outline"
                }
            }
        }
    }
}