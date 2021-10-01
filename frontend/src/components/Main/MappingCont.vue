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
export default {
  computed: {},
  date() {
    return {
      robot: [0, 0],
    };
  },
  methods: {
    poseRobot() {
      // if 서버 이미지를 받아온다면
      // y, x = 로봇의 목표 위치
      // sizeY, sizeX, resol = 이전 맵의 높이, 폭, 해상도
      //
      // const [y, x] = [30, 0];
      // const targetRobot = this.node2web(y, x);
      // const robot = document.querySelector("#robot");
      // // const [robotY, robotX] = [robot.offsetTop, robot.offsetLeft];
      // robot.style.top = targetRobot[0];
      // robot.style.left = targetRobot[1];console.log("ggg : ", robotDiv)
      // console.log("robot's cur coords : ", robotY, robotX);
    },
    node2web() {
      return [0, 0];
    },
  },
};
</script>
<style>
</style>