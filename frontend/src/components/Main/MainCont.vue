<template>
  <div id="MainWrapper">
    <Dashboard v-if="category_idx == 0"></Dashboard>
    <lost v-if="category_idx == 1"></lost>
    <history v-if="category_idx == 2"></history>
    <Picture v-if="category_idx == 3"></Picture>
    <Blackbox v-if="category_idx == 4"></Blackbox>
  </div>
</template>
<script>
import { mapState } from "vuex";
import "@/assets/css/MainWrapper.css";
import Dashboard from "./DashboardCont.vue";
import Lost from "./LostCont.vue";
import History from "./HistoryCont.vue";
import Picture from "./HistoryPicture.vue";
import Blackbox from "./BlackBox.vue";
export default {
  computed: {
    ...mapState(["category_idx"]),
  },
  components: {
    Lost,
    Dashboard,
    History,
    Picture,
    Blackbox
  },
};
</script>