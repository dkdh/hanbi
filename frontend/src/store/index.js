import Vue from "vue";
import Vuex from "vuex";
Vue.use(Vuex);

export default new Vuex.Store({
    state: {
        category_idx: 0
    },
    mutations: {
        setCategory(state, a) {
            // console.log("hi : ", a, state)
            state.category_idx = a
        }
    },
    actions: {},
    getters: {},
    modules: {
    },
});
