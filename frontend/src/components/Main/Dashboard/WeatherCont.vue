<template>
  <el-card
    id="WeatherCont"
    shadow="always"
  >
    <div slot="header">
      <span>날씨</span>
    </div>
    <!-- card body -->
    <div id="weatherWrap" style="text-align: center">
      <el-image :src="weatherSrc" fit="contain" :lazy="true"></el-image>
      <h3>{{ temperature }}℃ / {{ weather }}</h3>
        
    </div>
  </el-card>
</template>

<script>
import "@/assets/css_kjh/Dashboard/WeatherCont.css";
import { mapState,mapMutations } from "vuex";
export default {
  computed: {
    ...mapState("Environment", ["weather","weatherSrc", "temperature"]),
  },
  methods: {
    ...mapMutations("Environment", ["matchIcon"])
  },
  data() {
    return {
      // w : "@/assets/images/weather/039-sun.png"
      // w : require("./039-sun.png")
      // w : require("./039-sun.png")
      w : this.matchIcon("ㄹㄹ")
    }
  }
};
</script>

<style>
</style>