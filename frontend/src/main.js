import Vue from 'vue'
import App from './App.vue'
import router from './router'
import Vuex from 'vuex'
import ElementUI from 'element-ui';
import store from './store';
import 'element-ui/lib/theme-chalk/index.css';
import "./js/config"

Vue.config.productionTip = false

Vue.use(ElementUI)
Vue.use(Vuex)

new Vue({
  store,
  router,
  render: h => h(App)
}).$mount('#app')
