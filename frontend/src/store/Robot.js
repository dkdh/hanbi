export default {
    namespaced: true,
    state: {
        pos: [0, 0], battery: 100, velocity: 100, mode: "순찰모드 from"
    },
    mutations: {
        setRobot(state, data) {
            const { battery, velocity, pos, mode } = data
            state.battery = battery
            state.velocity = velocity
            state.pos = pos
            state.mode = mode
        }
    },
    actions: {
        setRobot({ state, commit, rootState }, data) {
            state

            const { pos } = data
            const [x, y] = pos
            let robotDiv = document.querySelectorAll(".robot")
            // console.log("set Robot : ", rootState.map.dsizeY - y, x)
            for (let i = 0; i < robotDiv.length; i++) {
                robotDiv[i].style.top = (rootState.map.dsizeY - y) + "px";
                robotDiv[i].style.left = x + "px";
                robotDiv[i].style.backgroundColor = rootState.colors.robot
            }
            commit("setRobot", data)
        }
    }
}