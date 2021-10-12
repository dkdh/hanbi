<template>
  <!-- <div> -->
  <el-card id="MapCont" shadow="always" :body-style="{ margin: '20px' }">
    <div slot="header">
      <h2>지도 및 위치</h2>
    </div>

    <div class="mapWrap">
      <div class="map">
        <img
          :src="require(`@/assets/map.png`)"
          width="500px"
          height="500px"
          z-index="5"
          v-on:click="clickFunc"
        />

        <!-- 로봇 -->
        <el-button
          class="onTheMap robot"
          type="primary"
          icon="el-icon-user"
          circle
        ></el-button>

        <!-- 이벤트 -->
        <el-button
          class="onTheMap event"
          :id="e.name"
          v-for="e in log"
          :key="e.name"
          circle
        >
          <i class="el-icon-circle-check"></i>
        </el-button>

        <!-- 범례 -->
        <div class="legend">
          <el-badge id="bdg_event" :value="num['event']" class="item">
            <el-button size="small"> event </el-button>
          </el-badge>

          <el-badge id="bdg_emergency" :value="num['emergency']" class="item">
            <el-button size="small"> emergency </el-button>
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
      interval3: 0,
    };
  },
  computed: {
    ...mapState(["colors"]),
    ...mapState("Log", ["log", "num"]),
  },
  methods: {
    clickFunc: (e) => {
      // console.log(e.offsetX, e.offsetY);
      store.dispatch("Map/click", [e.offsetX, e.offsetY]);
    },
  },
  mounted() {
    console.log("mounted");
    if (store.socket == null) return;
    store.socket.emit("History2Web");
    store.dispatch("Map/renderLog");
    //event 개수 새기
    this.interval3 = setInterval(() => {
      store.state.socket.emit("Robot2Web");
    }, 200);
  },
  updated() {
    store.dispatch("Map/renderLog");
  },
  beforeDestroy() {
    clearInterval(this.interval3);
  },
};
</script>
<style>
</style>