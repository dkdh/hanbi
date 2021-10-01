<template>
  <div id="DashboardCont">
    <div id="c1">
      <div id="r1">
        <div></div>
        <div></div>
        <div></div>
        <div></div>
      </div>
      <div id="r2">
        <map-view></map-view>
        <Blackbox></Blackbox>
        <log-view></log-view>
      </div>
    </div>
  </div>
</template>
<script>
import LogView from "./LogCont.vue";
import MapView from "./MapCont.vue";
import "@/assets/css/DashboardCont.css";
import Blackbox from "./BlackboxCont.vue";
export default {
  components: { LogView, MapView, Blackbox },
};
</script>

<style>
</style>