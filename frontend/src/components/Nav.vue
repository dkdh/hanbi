<template>
  <el-menu
    id="Nav"
    :default-active="activeIndex"
    class="el-menu-demo"
    mode="horizontal"
    style="background-color: #f5f5dc;"
  >
    <el-menu-item id="img" style="margin-left: 30px; margin-right: auto; color:black; background-color: #f5f5dc;">
      <h1>한비</h1>
    </el-menu-item>

    <div></div>

  </el-menu>
</template>

<script>
import "../assets/css/Nav.css";
export default {
  data() {
    return {
      activeIndex: "1",
      activeIndex2: "1",
    };
  },
  methods: {},
};
</script>

<style>
</style>