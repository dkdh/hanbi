<template>
  <div id="Main" class="bc" style="height: 100%">
    <Nav></Nav>
    <div id="mainView">
      <side-bar></side-bar>
      <main-wrapper></main-wrapper>
    </div>
  </div>
</template>
<script>
import Nav from "../components/Nav.vue";
import MainWrapper from "../components/Main/MainWrapper.vue";
import SideBar from "../components/Sidebar.vue";
import "../assets/css/Main.css";
import { mapState } from "vuex";
export default {
  computed: {
    ...mapState("a"),
  },
  components: {
    SideBar,
    Nav,
    MainWrapper,
  },
};
</script>