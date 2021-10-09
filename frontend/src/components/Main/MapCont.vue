<template>
  <!-- <div> -->
  <el-card id="MapCont" shadow="always" :body-style="{ margin: '20px' }">
    <div slot="header">
      <h2>지도 및 위치</h2>
    </div>
    <div class="mapWrap">
      <div class="map">
        <!-- !이미지 렌더링이 안됨 -->
        <img
          :src="require(`@/assets/map.png`)"
          width="500px"
          height="500px"
          z-index="5"
          v-on:click="clickFunc"
        />

        <div
          class="robot"
          style="width: 10px; height: 10px; background-color: red"
        ></div>
      </div>
    </div>
    <!-- card body -->
  </el-card>
  <!-- </div> -->
</template>

<script>
import "@/assets/css_kjh/MapCont.css";
import store from "@/store";
export default {
  data() {
    return {
      robot: [0, 0],
    };
  },
  methods: {
    clickFunc: (e) => {
      // console.log(e.offsetX, e.offsetY);
      store.dispatch("Map/convertCoords", [e.offsetX, e.offsetY]);
    },
  },
  mounted() {},
};
</script>
<style>
</style>