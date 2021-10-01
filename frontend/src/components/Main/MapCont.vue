<template>
  <!-- <div> -->
  <el-card id="MapCont" shadow="always" :body-style="{ margin: '20px' }">
    <div slot="header">
      <span>Map</span>
    </div>
    <div id="mapWapper">
      <div id="map">
        <!-- <img id="mapImg" src="@/assets/map_before.png" /> -->
        <canvas id="mapImg" width="500px" height="500px"> </canvas>
        <div
          id="robot"
          style="width: 10px; height: 10px; background-color: red"
        ></div>
      </div>
      <el-button type="primary" size="default" @click="poseRobot"></el-button>
    </div>
    <!-- card body -->
  </el-card>
  <!-- </div> -->
</template>

<script>
import "@/assets/css/MapCont.css";
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