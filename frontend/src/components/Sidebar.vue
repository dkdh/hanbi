<<<<<<< HEAD
<template>
  <el-aside
    id="Sidebar"
    style="width: 150px; font-size: 12px; text-align: center"
  >
    <el-card style="width: 150px">
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
      category: {
        0: "상황판",
        1: "분실물 수거",
        2: "로그",
        3: "로그 사진",
        4: "블랙박스 영상",
      },
    };
  },
  methods: {
    onClick: (idx) => {
      console.log(idx);
      store.commit("setCategory", idx);
      store.commit("Loading/On");
    },
  },
};
</script>
<style>
=======
<template>
  <el-aside
    id="Sidebar"
    style="width: 150px; font-size: 12px; text-align: center"
  >
    <el-card style="width: 150px">
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
      category: {
        0: "상황판",
        1: "분실물 수거",
        2: "로그",
        3: "로그 사진",
        4: "블랙박스 영상",
      },
    };
  },
  methods: {
    onClick: (idx) => {
      console.log(idx);
      store.commit("setCategory", idx);
      store.commit("Loading/On");
    },
  },
};
</script>
<style>
>>>>>>> 12401cc988d66fe8803686ebc605b2985adced9e
</style>