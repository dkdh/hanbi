<template>
  <el-card shadow="always" :body-style="{ padding: '20px' }">
    <div slot="header">
      <span>Status</span>
    </div>
    <h3>battery : {{ battery }}</h3>
    <h3>velocity : {{ velocity }}</h3>
    <h3>mode</h3>
    <!-- card body -->
  </el-card>
</template>

<script>
import { mapState } from "vuex";
export default {
  computed: {
    ...mapState("Robot", ["battery", "velocity"]),
  },
};
</script>

<style>
</style>