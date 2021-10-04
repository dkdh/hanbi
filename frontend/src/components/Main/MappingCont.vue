<template>
  <!-- <div> -->
  <el-card id="MappingCont" shadow="always" :body-style="{ margin: '20px' }">
    <div slot="header">
      <span>Mapping</span>
    </div>
    <div class="mapWrap">
      <div class="map">
        <!-- <img class="mapImg" src="@/assets/map_before.png" /> -->
        <canvas class="mappingImg" width="500px" height="500px"> </canvas>
        <!-- ! Canvas 안에 들어가야하는데 안됨 -->
        <div class="robot"></div>
      </div>
      <el-button type="primary" size="default" @click="poseRobot"></el-button>
    </div>
    <!-- card body -->
  </el-card>
  <!-- </div> -->
</template>

<script>
import "@/assets/css/MappingCont.css";
import { mapMutations } from "vuex";
export default {
  computed: {},
  date() {
    return {
      robot: [0, 0],
    };
  },
  methods: {
    ...mapMutations("Map", ["drawMapping"]),
  },
  mounted() {
    setInterval(() => {
      this.drawMapping();
    }, 1000);
  },
};
</script>
<style>
</style>