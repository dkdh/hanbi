import Vue from "vue";
import Vuex from "vuex";
Vue.use(Vuex);

export default new Vuex.Store({
    state: {
        category: 0
    },
    mutations: {
        setCategory(state, a) {
            // console.log("hi : ", a, state)
            state.category = a
        }
    },
    actions: {},
    getters: {},
    modules: {
    },
});
