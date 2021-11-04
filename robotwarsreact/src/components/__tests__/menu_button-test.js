import TestRenderer from "react-test-renderer";
import MenuButton from '../menu_button';

it("Renders correctly", () => {
  const renderer = TestRenderer.create(<MenuButton/>);
  expect(renderer.toJSON()).toMatchSnapshot();
});