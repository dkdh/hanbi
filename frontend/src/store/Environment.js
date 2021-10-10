export default {
    namespaced: true,
    state: {
        temperature: 30,
        weather: "sunny",
        weatherSrc : require('@/assets/images/weather/039-sun.png')
    },
    mutations: {
        matchIcon(state, weatherData) {
            if (weatherData === "Sunny") state.weatherSrc= require('@/assets/images/weather/039-sun.png');
            if (weatherData === "Cloudy") state.weatherSrc= require("@/assets/images/weather/001-cloud.png");
            if (weatherData === "Foggy") state.weatherSrc= require("@/assets/images/weather/017-foog.png");
            if (weatherData === "Storm") state.weatherSrc= require("@/assets/images/weather/047-tornado-1.png");
            if (weatherData === "Rainy") state.weatherSrc= require("@/assets/images/weather/003-rainy.png");
            if (weatherData === "Snowy") state.weatherSrc= require("@/assets/images/weather/006-snowy.png");
        },
        // renderWeather(state, weatherData) {
            // const img = document.querySelector("#weatherImg");
            // if(!img) return
            // matchIcon(weatherData)
        // }
    },
    actions: {
        setEnvironment({state, commit}, data) {
            const { temperature, weather } = data
            state.temperature = temperature
            if(state.weather != weather) {
                state.weather = weather
                commit("matchIcon",weather)
                // commit("renderWeather", weather)
            }
        },
    }
}