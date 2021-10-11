export default {
    namespaced: true,
    state: {
        pos: [0, 0], battery: 100, velocity: 100, mode: "순찰모드 from"
    },
    mutations: {
        setRobot(state, data) {
            //Main에 로봇 상태를 나타내는 뮤테이션
            const { battery, velocity, pos, mode } = data
            state.battery = battery
            state.velocity = velocity*25
            state.pos = pos
            state.mode = mode
        }
    },
    actions: {
        setRobot({  commit, rootState }, data) {
            //Map에 로봇을 그리는 액션
            
            //현재 객체 트리에 로봇이 없으면 반환
            let robotDiv = document.querySelectorAll(".robot")
            if(!robotDiv) return 

            const { pos } = data
            const [x, y] = pos
            // console.log("set Robot : ", rootState.Map.dSizeY - y, x)
            for (let i = 0; i < robotDiv.length; i++) {
                robotDiv[i].style.top = (rootState.Map.dSizeY - y) + "px";
                robotDiv[i].style.left = x + "px";
                // robotDiv[i].style.backgroundColor = rootState.colors.robot
            }
            commit("setRobot", data)
        }
    }
}