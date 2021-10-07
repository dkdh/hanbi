<template>
  <el-card id="WeatherCont" shadow="always" style="padding-left:140px; padding-right:140px">
    <div slot="header">
      <span>날씨</span>
    </div>
    <!-- card body -->
    <div id="weatherWrap" style="text-align: center">
      <h1>{{ temperature }}℃</h1>
      <div>
        <h1>{{ weather }}</h1>
      </div>
    </div>
  </el-card>
</template>

<script>
import "@/assets/css/Dashboard/WeatherCont.css";
import { mapState } from "vuex";
export default {
  computed: {
    ...mapState("Environment", ["weather", "temperature"]),
  },
};
</script>

<style>
</style>