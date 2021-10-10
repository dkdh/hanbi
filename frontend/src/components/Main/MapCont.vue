<template>
  <!-- <div> -->
  <el-card id="MapCont" shadow="always" :body-style="{ margin: '20px' }">
    <div slot="header">
      <h2>지도 및 위치 </h2>
    </div>

    <div class="mapWrap">
      <!-- <el-checkbox v-for="e in list" :key="e.name" :fill="e.value">{{
        e.name
      }}</el-checkbox> -->
      <div class="map">
        <img
          :src="require(`@/assets/map.png`)"
          width="500px"
          height="500px"
          z-index="5"
          v-on:click="clickFunc"
        />
        <div
          class="onTheMap robot"
        >
        </div>
        <div class="onTheMap event" :id="e.name" v-for="e in log" :key="e.name">

        </div>
        <div class="legend">
          <!-- <el-badge v-for="e in Object.keys(num)" :key="e.name" :value="1" class="item"> -->
          <el-badge id = "bdg_robot" :value='num["robot"]' class='item'>
            robot
          </el-badge>
          <el-badge id = "bdg_event" :value='num["event"]' class='item'>
            event
          </el-badge>
          <el-badge id = "bdg_emergency" :value='num["emergency"]' class='item'>
            emergency
          </el-badge>
        </div>
      </div>
    </div>
    <!-- {{log}} -->
    <!-- card body -->
  </el-card>
  <!-- </div> -->
</template>

<script>
import "@/assets/css_kjh/MapCont.css";
import store from "@/store";
import { mapState } from "vuex";
export default {
  data() {
    return {
      robot: [0, 0],
      list: [],
    };
  },
  computed: {
    ...mapState(["colors"]),
    ...mapState("Log", ["log", "num"])
  },
  methods: {
    clickFunc: (e) => {
      // console.log(e.offsetX, e.offsetY);
      store.dispatch("Map/click", [e.offsetX, e.offsetY]);
    },
  },
  mounted() {
    const colorlist = ["robot", "event", "emergency"];
    console.log(this.colors);
    for (let e in colorlist) {
      let name = colorlist[e];
      this.list.push({ name: name, value: this.colors[name]});
    }
    //event 개수 새기
  },
  updated() {
    store.dispatch("Log/renderLog")
  },
};
</script>
<style>
</style>