export default {
    namespaced: true,
    state: {
        is_load_map1: true,
        is_load_map2: true,
        is_load_robot: true,
        is_load_mapLog: true,
        is_load_WeatherCont: true,
        is_load_RobotCont: true
    },
    mutations: {
        On(state) {
            Object.keys(state).reduce((acc, cur) => {
                state[cur] = true
            })
        }
    },
    actions: {
    }
}