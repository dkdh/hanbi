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
        drawMapping(state) {
            console.log("drawMapping : ", state)
            
            const { dSizeY, dSizeX, data, colors } = state

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

                for (let i = 0; i < data.length; i++) {
                    if (data[i].length != 3) continue
                    const [y, x, v] = data[i]
                    const next = colors.find_pre + v.toString(16) + colors.find_suf

                    state.data[y][x] = next
                }

            } catch (e) {
                console.log("no mapImg", e)
            }
        },
        click({ state, rootState }, data) {
            //ROS에서 사용하는 좌표 정의
            const [x, y] = data
            const { dSizeX, dSizeY,resol } = state
            // const [rosSizeX, rosSizeY] = [350, 350]

            //변환
            const cvtX = (x - (dSizeX/ 2) * resol) 
            const cvtY = ((dSizeY / 2) * resol - y) 

            console.log(cvtX, cvtY)

            console.log(rootState.losts)
            const { socket } = rootState
            if (socket == null) throw Error("No socket In convertCoords")

            socket.emit("Click2Server", { x: cvtX, y: cvtY })
        },
        renderLog({state, rootState},data) {
            //map에 log 기록을 남기는 함수
            state, data, rootState
            const {dSizeY, dSizeX,resol} = state
            const {log} = rootState.Log

            let logDOM = document.querySelectorAll('.event');

            // 이벤트 개수 세기, 객체 설정
            for(let i=0; i <  logDOM.length;i++) {
                const {y,x} = log[i].pose
                y,x,dSizeY, dSizeX,resol
                logDOM[i].style.top = (y/resol + dSizeY/2 ) +"px"
                logDOM[i].style.left = (x/resol + dSizeX/2)+"px"
                if(log[i].emergency===1){
                    logDOM[i].style.backgroundColor = rootState.colors["emergency"]
                }else {
                    logDOM[i].style.backgroundColor = rootState.colors["event"]
                }
            }
        }
    }
}