<template>
  <el-menu
    id="Nav"
    :default-active="activeIndex"
    class="el-menu-demo"
    mode="horizontal"
  >
    <el-menu-item id="img" style="margin-left: 50px">
      <h1>한비</h1>
    </el-menu-item>
    <!-- <el-menu-item index="1">Hello, {{ email }}</el-menu-item> -->

    <el-button type="primary" size="default" style="margin: 5px 15px 5px auto">
      btn</el-button
    >
    <!-- <el-menu-item index="2" key=""> </el-menu-item> -->
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