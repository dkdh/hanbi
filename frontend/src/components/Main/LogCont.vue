<template>
  <!-- <div id="LogView" class="block"> -->
  <el-card id="LogCont" shadow="always" :body-style="{ padding: '20px' }">
    <div slot="header">
      <span>Action logs</span>
    </div>
    <!-- card body -->
    <div id="history">
      <div class="radio">
        Order:
        <el-radio-group v-model="reverse">
          <el-radio :label="true">descending</el-radio>
          <el-radio :label="false">ascending</el-radio>
        </el-radio-group>
      </div>
      <!-- {{ actions }} -->
      <el-timeline :reverse="reverse">
        <el-timeline-item
          v-for="(activity, index) in actions"
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
export default {
  // props: ["actions"],
  data() {
    return {
      reverse: true,
      actions: [
        {
          timestamp: new Date(),
          content: "hello",
        },
        {
          timestamp: new Date(),
          content: "hello",
        },
        {
          timestamp: new Date(),
          content: "hello",
        },
        {
          timestamp: new Date(),
          content: "hello",
        },
      ],
    };
  },
  mounted() {},
};
</script>

<style>
</style>