<template>
  <!-- <div id="LogView" class="block"> -->
  <el-card id="LogCont" shadow="always" :body-style="{ padding: '20px' }">
    <div slot="header">
      <h2>로그</h2>
    </div>
    <!-- card body -->
    <div id="history">
      <div class="radio">
        정렬:
        <el-radio-group v-model="reverse">
          <el-radio :label="true">오름차순</el-radio>
          <el-radio :label="false">내림차순</el-radio>
        </el-radio-group>
      </div>
      <!-- {{ actions }} -->
      <el-timeline :reverse="reverse">
        <el-timeline-item
          v-for="(activity, index) in log"
          :key="index"
          :timestamp="activity.timestamp"
        >
          {{ activity.content }}
        </el-timeline-item>
      </el-timeline>
    </div>
  </el-card>

  <!-- {{ actions }} -->
  <!-- </div> -->
</template>

<script>
import "@/assets/css/LogView.css";
import { mapState } from "vuex";
export default {
  // props: ["actions"],
  computed: {
    ...mapState(["log"]),
    ...mapState("Map", {
      data: (state) => state.data,
    }),
  },
  data() {
    return {
      reverse: true,
    };
  },
  mounted() {},
};
</script>

<style>
</style>