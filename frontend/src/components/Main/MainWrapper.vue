<template>
  <div id="MainWrapper">
    {{ category_idx }}
    {{ category_idx == 0 }}
    <Map v-if="category_idx == 0"></Map>
    <lost v-if="category_idx == 1"></lost>
    <log v-if="category_idx == 2"></log>
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
    ...mapState(["category_idx"]),
  },
  components: {
    Log,
    Lost,
    Map,
  },
};
</script>