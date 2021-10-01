export default {
    namespaced: true,
    state: {
        temperature: 30,
        weather: "sunny"
    },
    mutations: {
        setEnvironment(state, data) {
            const { temperature, weather } = data
            state.temperature = temperature
            state.weather = weather
        }
    },
    actions: {
    }
}