export default {
    namespaced: true,
    state: {
        log: [
            { timestamp: String(new Date()), content: "vuex" },
            { timestamp: String(new Date()), content: "vuex2" },
            { timestamp: String(new Date()), content: "vuex3" }
        ]
    },
    mutations: {
    },
    actions: {
        setLog({ state }, data) {
            state, data
            state.log = data
        }
    }
}