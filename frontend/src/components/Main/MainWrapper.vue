<template>
  <div id="MainWrapper">
    <Map v-if="category == 0"></Map>
    <lost v-if="category == 1"></lost>
    <log v-if="category == '2'"></log>
  </div>
</template>
<script>
import { mapState } from "vuex";
import "../../assets/css/MainWrapper.css";
import Log from "./LogView.vue";
import Lost from "./LostView.vue";
import Map from "./MapView.vue";
export default {
  computed: {
    ...mapState(["category"]),
  },
  components: {
    Log,
    Lost,
    Map,
  },
};
</script>