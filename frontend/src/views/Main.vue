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
import { mapActions } from "vuex";
import "../assets/css/Main.css";
export default {
  computed: {},
  components: {
    SideBar,
    Nav,
    MainWrapper,
  },
  methods: {
    ...mapActions(["setSockets"]),
  },
  created() {
    this.setSockets();
  },
};
</script>