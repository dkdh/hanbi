<template>
  <el-aside id="Sidebar">
    <el-card>
      {{ category_idx }}
      <a v-for="(e, idx) in category" :key="e" @click="onClick(idx)">
        <h1>{{ e }}</h1></a
      >
    </el-card>
  </el-aside>
</template>

<script>
// import { mapState } from "vuex";
import "../assets/css/Sidebar.css";
import store from "@/store";
import { mapState } from "vuex";
export default {
  computed: {
    ...mapState(["category_idx"]),
  },
  data() {
    return {
      category: { 0: "map", 1: "lost and found", 2: "log" },
    };
  },
  methods: {
    onClick: (idx) => {
      console.log(idx);
      store.commit("setCategory", idx);
    },
  },
};
</script>
<style>
</style>