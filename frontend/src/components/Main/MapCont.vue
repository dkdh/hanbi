<template>
  <!-- <div> -->
  <el-card id="MapCont" shadow="always" :body-style="{ margin: '20px' }">
    <div slot="header">
      <span>Map</span>
    </div>
    <div class="mapWrap">
      <div class="map">
        <!-- !이미지 렌더링이 안됨 -->
        <img
          :src="require(`@/assets/map.png`)"
          width="500px"
          height="500px"
          z-index="5"
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
import "@/assets/css/MapCont.css";
export default {
  computed: {},
  date() {
    return {
      robot: [0, 0],
    };
  },
  methods: {},
  mounted() {},
};
</script>
<style>
</style>