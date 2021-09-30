<template>
  <div id="LostView">
    <!-- 카드 컴포넌트를 나열 -->
    <div id="lostWrapper">
      <el-card
        v-for="(e, idx) in losts"
        :key="idx"
        :body-style="{ padding: '0px' }"
      >
        <img
          src="https://shadow.elemecdn.com/app/element/hamburger.9cf7b091-55e9-11e9-a976-7f4d0b07eef6.png"
          class="image"
        />
        <div style="padding: 14px">
          <span>{{ e.name }}</span>
          <div class="bottom clearfix" style="display: flex">
            <time class="time">{{ e.date }}</time>
            <el-button type="text" class="button">Operating</el-button>
          </div>
        </div>
      </el-card>
    </div>
  </div>
</template>

<script>
// import store from "@/store";
import { mapState } from "vuex";
import "@/assets/css/LostView.css";
export default {
  computed: {
    ...mapState(["losts"]),
  },
};
</script>
<style>
</style>