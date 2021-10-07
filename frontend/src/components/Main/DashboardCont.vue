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
        <div id="c3">
          <weather-cont id="c4"></weather-cont>
          <robot-cont id="c5"></robot-cont>
        </div>
        <monitoring-cont id="c6"></monitoring-cont>
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
import MonitoringCont from "./Dashboard/MonitoringCont.vue";

export default {
  components: {
    // BlackboxCont,
    // LogCont,
    // MappingCont,
    MapCont,
    WeatherCont,
    RobotCont,
    MonitoringCont,
  },
};
</script>

<style>
</style>