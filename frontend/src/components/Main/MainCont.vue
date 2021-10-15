<template>
  <div id="MainWrapper" >
    <Dashboard v-if="category_idx == 0"></Dashboard>
    <lost v-if="category_idx == 1"></lost>
    <history v-if="category_idx == 2"></history>
    <LogPhoto v-if="category_idx == 3"></LogPhoto>
    <Blackbox v-if="category_idx == 4"></Blackbox>
  </div>
</template>
<script>
import { mapState } from "vuex";
import "@/assets/css/MainWrapper.css";
import Dashboard from "./DashboardCont.vue";
import Lost from "./LostCont.vue";
import History from "./HistoryCont.vue";
import LogPhoto from "./LogPhotoCont.vue"
import Blackbox from "./BlackboxCont.vue";
export default {
  computed: {
    ...mapState(["category_idx"]),
  },
  components: {
    Lost,
    Dashboard,
    History,
    LogPhoto,
    Blackbox,
  },
  
};
</script>