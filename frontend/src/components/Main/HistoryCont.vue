<template>
  <div id="HistoryCont">
    <div id="historyWrap">
      <mapping-cont></mapping-cont>
      <log-cont></log-cont>
    </div>
  </div>
</template>

<script>
import "@/assets/css/HistoryCont.css";
import LogCont from "@/components/Main/History/LogCont.vue";
import MappingCont from "./MappingCont.vue";
export default {
  components: { LogCont, MappingCont },
};
</script>

<style>
</style>