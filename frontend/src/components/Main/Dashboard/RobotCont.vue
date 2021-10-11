<template>
  <el-card
    shadow="always"
  >
    <div slot="header">
      <span style="margin-right:15px">한비 상태</span>
    <el-tag type="success">{{ mode_name[mode] }}</el-tag>
    </div>
    <!-- <h3>battery {{ battery }}</h3> -->
    <!-- <h3>velocity {{ velocity }}</h3> -->
    <div style="display:flex;justify-content:center;align-items:center">
    <div  style="margin-right:25px">
    <el-progress type="dashboard" :percentage="battery" :color="colors"></el-progress>
    <h3>battery</h3>
    </div>
    <div>
    <el-progress type="dashboard" :percentage="velocity" :color="colors"></el-progress>
    <h3>velocity</h3>
    </div>
    </div>
    
    <!-- card body -->
  </el-card>
</template>

<script>
import { mapState } from "vuex";
export default {
  computed: {
    ...mapState("Robot", ["battery", "velocity", "mode"]),
  },
  data() {
    return {
       mode_name: { 0: "경비모드", 1: "방역모드" } ,
       percentage: 10,
        colors: [
          {color: '#f56c6c', percentage: 20},
          {color: '#e6a23c', percentage: 40},
          {color: '#5cb87a', percentage: 60},
          {color: '#1989fa', percentage: 80},
          {color: '#6f7ad3', percentage: 100}
        ]
       };
  },
};
</script>

<style>
</style>