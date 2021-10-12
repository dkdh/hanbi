<template>
  <!-- <div> -->
  <el-card id="MappingCont" shadow="always" :body-style="{ margin: '20px' }">
    <div slot="header">
      <h2>맵 감지 현황</h2>
    </div>
    <div class="mapWrap">
      <div
        class="map"
        v-loading="is_load_map2"
        element-loading-spinner="el-icon-loading"
      >
        <el-progress
          id="mappingProgressBar"
          :percentage="Math.floor((percentage / dSize) * 100)"
        >
        </el-progress>
        <!-- <img class="mapImg" src="@/assets/map_before.png" /> -->
        <canvas class="mappingImg" width="500px" height="500px"> </canvas>

        <el-button
          class="onTheMap robot"
          type="primary"
          icon="el-icon-user"
          circle
        ></el-button>
      </div>
    </div>
    <!-- card body -->
  </el-card>
  <!-- </div> -->
</template>

<script>
import "@/assets/css_kjh/MappingCont.css";
import { mapMutations, mapState } from "vuex";
import store from "@/store";
export default {
  computed: {
    ...mapState("Map", ["percentage", "dSize"]),
    ...mapState("Loading", ["is_load_map2"]),
  },
  date() {
    return {
      robot: [0, 0],
      customColors: [
        { color: "#f56c6c", percentage: 20 },
        { color: "#e6a23c", percentage: 40 },
        { color: "#5cb87a", percentage: 60 },
        { color: "#1989fa", percentage: 80 },
        { color: "#6f7ad3", percentage: 100 },
      ],
      interval1: 0,
      interval2: 0,
      interval3: 0,
    };
  },
  methods: {
    ...mapMutations("Map", ["drawMapping"]),
  },
  mounted() {
    store.state.socket.emit("MapInit");
    store.dispatch("Map/getMapInterval");
    store.dispatch("Map/drawMapInterval");
    store.dispatch("Robot/setRobotInterval");
  },
  beforeDestroy() {
    store.dispatch("Map/stopMapInterval");
    store.dispatch("Map/stopDrawMapInterval");
    store.dispatch("Robot/stopSetRobotInterval");
  },
};
</script>
<style>
</style>