import { extractField, formatValue, parsePath } from "../src/renderer/utils/field-extract";

describe("parsePath", () => {
  test("simple key", () => {
    expect(parsePath("x")).toEqual(["x"]);
  });

  test("dot notation", () => {
    expect(parsePath("linear_acceleration.x")).toEqual(["linear_acceleration", "x"]);
  });

  test("array index", () => {
    expect(parsePath("position[0]")).toEqual(["position", 0]);
  });

  test("deep path with array", () => {
    expect(parsePath("twist.twist.linear.x")).toEqual(["twist", "twist", "linear", "x"]);
  });

  test("array only", () => {
    expect(parsePath("[2]")).toEqual([2]);
  });
});

describe("extractField", () => {
  const msg = {
    linear_acceleration: { x: 1.5, y: -0.3, z: 9.8 },
    angular_velocity: { z: 0.1 },
    position: [0.0, 1.0, 2.0],
    twist: { twist: { linear: { x: 0.5 } } },
  } as Record<string, unknown>;

  test("simple nested field", () => {
    expect(extractField(msg, "linear_acceleration.x")).toBeCloseTo(1.5);
  });

  test("array index", () => {
    expect(extractField(msg, "position[1]")).toBeCloseTo(1.0);
  });

  test("deep nested", () => {
    expect(extractField(msg, "twist.twist.linear.x")).toBeCloseTo(0.5);
  });

  test("missing field returns null", () => {
    expect(extractField(msg, "missing.field")).toBeNull();
  });

  test("empty object returns null", () => {
    expect(extractField({}, "any")).toBeNull();
  });

  test("empty path returns null", () => {
    expect(extractField(msg, "")).toBeNull();
  });

  test("out of bounds array index returns null", () => {
    expect(extractField(msg, "position[99]")).toBeNull();
  });

  test("non-numeric field returns null", () => {
    const data = { label: "test" } as Record<string, unknown>;
    expect(extractField(data, "label")).toBeNull();
  });
});

describe("formatValue", () => {
  test(".2f format", () => {
    expect(formatValue(3.14159, ".2f")).toBe("3.14");
  });

  test(".6f format", () => {
    expect(formatValue(51.12345678, ".6f")).toBe("51.123457");
  });

  test(".0f format", () => {
    expect(formatValue(42.7, ".0f")).toBe("43");
  });

  test("no format returns string of number", () => {
    expect(formatValue(5)).toBe("5");
  });

  test("unit appended", () => {
    expect(formatValue(1.23, ".2f", "m/s")).toBe("1.23 m/s");
  });

  test("unit without format", () => {
    expect(formatValue(100, undefined, "Hz")).toBe("100 Hz");
  });
});
