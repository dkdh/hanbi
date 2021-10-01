<template>
  <el-card id="WeatherCont" shadow="always" :body-style="{ padding: '20px' }">
    <div slot="header">
      <span>Weather</span>
    </div>
    <!-- card body -->
    <div id="weatherWrap" style="">
      <h1>{{ temperature }}</h1>
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