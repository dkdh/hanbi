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
    }
}