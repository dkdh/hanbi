<template>
  <!-- <div id="BlackboxContainer" class="block"> -->
  <el-card id="BlackboxCont" shadow="always" :body-style="{ padding: '20px' }">
    <div slot="header">
      <span>Camera</span>
    </div>
    <div id="BalckboxWrap">
      <div id="camView">
        <h1>승철햄,,!</h1>
        <h3>blackbox/socket.io</h3>
      </div>
    </div>
  </el-card>
  <!-- {{ actions }} -->
  <!-- </div> -->
</template>

<script>
import "@/assets/css/BlackboxCont.css";
export default {
  // props: ["actions"],
  data() {
    return {
      reverse: true,
      actions: [
        {
          timestamp: new Date(),
          content: "hello",
        },
        {
          timestamp: new Date(),
          content: "hello",
        },
        {
          timestamp: new Date(),
          content: "hello",
        },
        {
          timestamp: new Date(),
          content: "hello",
        },
      ],
    };
  },
  mounted() {},
};
</script>

<style>
</style>