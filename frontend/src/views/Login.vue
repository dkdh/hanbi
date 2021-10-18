<template>
  <div class="about">
<<<<<<< HEAD
    <h2>비밀번호를 입력해주세요</h2>
=======
    <h2>비밀번호를 입력해주세요(0310)</h2>
>>>>>>> 12401cc988d66fe8803686ebc605b2985adced9e
    <input type="password" v-model="password" @keyup.enter="login">
    <button @click="login">Login</button>
  </div>
</template>


<script>
export default {
  name: 'Login',
  data() {
    return {
      password: ''
    }
  },
  methods: {
    login() {
      if(this.password == '0310') {
        sessionStorage.setItem('password', this.password);
        this.$router.push({ name: 'Main' });
      }
    }
  },
}
</script>