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
      <el-timeline :reverse="reverse">
        <el-timeline-item v-for="(activity, index) in log" :key="index">
          <!-- normal event -->
          <el-card v-if="activity.emergency == 0">
            <h4>{{ activity.content }}</h4>
            <p>{{ activity.timestamp }}</p>
          </el-card>
          <!-- normal event -->
          <el-card
            v-if="activity.emergency == 1"
            style="background-color: #f56c6c"
          >
            <h4>{{ activity.content }}</h4>
            <p>{{ activity.timestamp }}</p>
          </el-card>
        </el-timeline-item>
      </el-timeline>
    </div>
  </el-card>
  <!-- </div> -->
</template>

<script>
import "@/assets/css_kjh/LogView.css";
import { mapState } from "vuex";
export default {
  computed: {
    ...mapState("Log", {
      log: (state) => state.log,
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