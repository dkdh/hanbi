<template>
  <el-card shadow="always" :body-style="{ padding: '20px' }">
    <div slot="header">
      <span>Status</span>
    </div>
    <h3>battery : {{ battery }}</h3>
    <h3>velocity : {{ velocity }}</h3>
    <h3>mode : {{ mode_name[mode] }}</h3>
    <!-- card body -->
  </el-card>
</template>

<script>
import { mapState } from "vuex";
export default {
  computed: {
    ...mapState("Robot", ["battery", "velocity", "mode"]),
  },
  data() {
    return { mode_name: { 0: "경비모드", 1: "방역모드" } };
  },
};
</script>

<style>
</style>