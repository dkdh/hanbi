import Vue from 'vue'
import VueRouter from 'vue-router'
import Main from "../views/Main.vue"
import Login from "../views/Login.vue"


Vue.use(VueRouter)

const routes = [
  {
    path: '/',
    name: 'Login',
    component: Login,
    // 로그인 한 적이 있으면 login 페이지에 접근해도 main으로 바로 이동
    beforeEnter: (to, from, next) => {
      if (!sessionStorage.getItem('password')) {
        return next();
      }
      next('/main');
    }
  },
  {
    path: '/main',
    name: 'Main',
    component: Main,
    // 로그인 안 했으면 로그인 페이지로 이동
    beforeEnter: (to, from, next) => {
      if (sessionStorage.getItem('password')) {
        return next();
      }
      next('/');
    }
  }
]

const router = new VueRouter({
  mode: 'history',
  base: process.env.BASE_URL,
  routes
})

export default router
