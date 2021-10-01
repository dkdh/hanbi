<template>
  <div id="MainWrapper">
    <Dashboard v-if="category_idx == 0"></Dashboard>
    <lost v-if="category_idx == 1"></lost>
  </div>
</template>
<script>
import { mapState } from "vuex";
import "@/assets/css/MainWrapper.css";
import Dashboard from "./DashboardCont.vue";
import Lost from "./LostCont.vue";
export default {
  computed: {
    ...mapState(["category_idx"]),
  },
  components: {
    Lost,
    Dashboard,
  },
};
</script>