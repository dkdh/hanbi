<template>
  <!-- <div> -->
  <el-card id="MapCont" shadow="always" :body-style="{ margin: '20px' }">
    <div slot="header">
      <h2>지도 및 위치</h2>
    </div>

    <div class="mapWrap">
      <div
        class="map"
        v-loading="is_load_mapLog"
        element-loading-spinner="el-icon-loading"
      >
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
  </el-card>
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
      loading: true,
    };
  },
  computed: {
    ...mapState(["colors"]),
    ...mapState("Log", ["log", "num"]),
    ...mapState("Loading", ["is_load_map1", "is_load_mapLog"]),
  },
  methods: {
    clickFunc: (e) => {
      store.dispatch("Map/click", [e.offsetX, e.offsetY]);
    },
  },
  mounted() {
    //pulling
    store.dispatch("Log/getLogInterval");
    store.dispatch("Map/renderLogInterval");
    store.dispatch("Robot/setRobotInterval");
  },

  beforeUpdate() {
    console.log(this.is_load_map1, this.is_load_mapLog);
  },
  beforeDestroy() {
    store.dispatch("Log/stopLogInterval");
    store.dispatch("Map/stopRenderLogInterval");
    store.dispatch("Robot/stopSetRobotInterval");
  },
};
</script>
<style>
</style>