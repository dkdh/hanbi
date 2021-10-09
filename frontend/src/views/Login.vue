<template>
  <div class="about">
    <h2>비밀번호를 입력해주세요</h2>
    <input type="password" v-model="password">
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