<template>
  <div id="DashboardCont">
    <div id="DashboardWrap">
      <div id="c1">
        <!-- <div id="r1">
        <div></div>
        <blackbox-cont></blackbox-cont>
      </div> -->
        <div id="r2">
          <map-cont></map-cont>
          <!-- <mapping-cont></mapping-cont> -->
          <!-- <log-cont></log-cont> -->
        </div>
      </div>
      <div id="c2">
        <weather-cont></weather-cont>
        <robot-cont></robot-cont>
      </div>
    </div>
  </div>
</template>
<script>
// import LogCont from "./LogCont.vue";
// import MappingCont from "./MappingCont.vue";
import MapCont from "./MapCont.vue";
import "@/assets/css/DashboardCont.css";
// import BlackboxCont from "./BlackboxCont.vue";
import WeatherCont from "./Dashboard/WeatherCont.vue";
import RobotCont from "./Dashboard/RobotCont.vue";

export default {
  components: {
    // BlackboxCont,
    // LogCont,
    // MappingCont,
    MapCont,
    WeatherCont,
    RobotCont,
  },
};
</script>

<style>
</style>