/**
 * Static Mock Responses for Hackathon Demo
 *
 * NO API KEY REQUIRED - All responses are pre-written
 *
 * This file contains:
 * 1. Full Urdu translations for key chapters
 * 2. Personalization tips for different user backgrounds
 * 3. Bulldog AI confirmation messages
 */

// ========================================
// URDU TRANSLATIONS (Task 7 - Complete)
// ========================================

export const URDU_TRANSLATIONS: Record<string, string> = {
  '/docs/intro': `# جسمانی AI کی درسی کتاب میں خوش آمدید

یہ جامع درسی کتاب جسمانی AI - روبوٹکس، کمپیوٹر ویژن، اور مجسم AI سسٹمز کی دنیا میں آپ کی رہنما ہے۔

## 📚 آپ کیا سیکھیں گے

- **ROS 2**: صنعتی معیار کا روبوٹ سافٹ ویئر فریم ورک
- **Simulation**: Gazebo، Unity، اور NVIDIA Isaac Sim میں ڈیجیٹل ٹوئنز
- **AI Perception**: NVIDIA Isaac کے ساتھ GPU سے تیز شدہ کمپیوٹر ویژن
- **VLA Models**: Vision-Language-Action ماڈلز قدرتی زبان کے تعامل کے لیے

## 🎯 یہ کس کے لیے ہے

یہ کورس ڈیزائن کیا گیا ہے:
- سافٹ ویئر انجینئرز جو روبوٹکس میں منتقل ہو رہے ہیں
- ہارڈویئر ماہرین AI کے ساتھ اپ گریڈ کر رہے ہیں
- طالب علم جدید روبوٹک سسٹمز تعمیر کرنا سیکھ رہے ہیں

## 🚀 شروع کریں

اپنے سفر کا آغاز کرنے کے لیے **ماڈیول 1: ROS 2 بنیادی باتیں** پر جائیں۔`,

  '/docs/module1/chapter1-1-ros2-fundamentals': `# باب 1.1: ROS 2 بنیادی باتیں

## تعارف

ROS 2 (Robot Operating System 2) روبوٹ سافٹ ویئر ڈویلپمنٹ کے لیے صنعتی معیار کا فریم ورک ہے۔ یہ ROS 1 کا اگلا ورژن ہے جس میں real-time performance، security، اور multi-robot systems کے لیے بہتری کی گئی ہے۔

## اہم تصورات

### Nodes - نوڈز

نوڈز آزاد پروسیسز ہیں جو کمپیوٹیشن انجام دیتے ہیں۔ ہر نوڈ ایک مخصوص کام کرتا ہے جیسے sensor data پڑھنا، motor control، یا path planning۔

**مثال**: ایک TurtleBot میں:
- \`/camera_node\` - کیمرہ سے تصاویر لیتا ہے
- \`/motor_controller\` - پہیوں کو کنٹرول کرتا ہے
- \`/navigation\` - راستہ منصوبہ بندی کرتا ہے

### Topics - ٹاپکس

ٹاپکس میسج پاسنگ کمیونیکیشن کے لیے نامزد چینلز ہیں۔ نوڈز topics پر publish اور subscribe کر سکتے ہیں۔

**مثال**:
\`\`\`bash
ros2 topic list
ros2 topic echo /scan
\`\`\`

### Services - سروسز

سروسز درخواست-جواب پیٹرن ہیں سنکرونس تعاملات کے لیے۔ Client ایک سروس کو call کرتا ہے اور response کا انتظار کرتا ہے۔

## Publisher-Subscriber Pattern

ROS 2 publisher-subscriber pattern استعمال کرتا ہے asynchronous کمیونیکیشن کے لیے۔

**فوائد**:
- Decoupling: نوڈز ایک دوسرے سے آزاد ہیں
- Scalability: آسانی سے نئے نوڈز شامل کریں
- Flexibility: real-time میں نوڈز start/stop کریں

## عملی مثال

یہاں ایک سادہ publisher node ہے:

\`\`\`python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2!'
        self.publisher_.publish(msg)
\`\`\``,

  '/docs/module1/intro': `# ماڈیول 1: روبوٹک اعصابی نظام (ROS 2)

## جائزہ

یہ ماڈیول ROS 2 کو کور کرتا ہے - روبوٹ سافٹ ویئر ڈویلپمنٹ کے لیے بنیاد۔ ROS 2 صنعت کا معیار ہے autonomous vehicles، manufacturing robots، اور research platforms کے لیے۔

## آپ کیا سیکھیں گے

1. **ROS 2 Architecture** اور core concepts
   - Nodes، topics، services، اور actions کی سمجھ
   - Communication patterns اور QoS policies

2. **Navigation Stack** اور path planning
   - SLAM algorithms
   - Obstacle avoidance
   - Motion planning

3. **Real-time Control Systems**
   - PID controllers
   - Sensor fusion
   - Feedback loops

## ضروریات

- **Python یا C++** کی بنیادی سمجھ
- **Linux Command Line** کی واقفیت
- **Robot Operating System** میں دلچسپی

## ماڈیول کی ساخت

| باب | عنوان | دورانیہ |
|-----|--------|----------|
| 1.1 | ROS 2 Fundamentals | 45 منٹ |
| 1.2 | Advanced Concepts | 60 منٹ |
| 1.3 | Navigation Stack | 90 منٹ |
| 1.4 | Control Systems | 75 منٹ |

## شروع کریں

پہلے باب سے شروع کریں: **[ROS 2 Fundamentals](./chapter1-1-ros2-fundamentals)**`,

  '/docs/module3/chapter3-3-isaac-manipulation-nav': `# باب 3.3: Isaac Manipulation اور Navigation

**ماڈیول**: 3 - AI-Robot Brain
**ہفتہ**: 11
**مطالعہ کا وقت**: 30 منٹ

---

## سیکھنے کے مقاصد

اس باب کے اختتام تک، آپ قابل ہوں گے:

1. خودمختار navigation کے لیے Isaac Visual SLAM کو Nav2 کے ساتھ ضم کریں
2. GPU سے تیز شدہ motion planning کے لیے cuMotion استعمال کریں
3. موبائل روبوٹس کے لیے costmaps اور planners ترتیب دیں
4. خودمختار waypoint navigation لاگو کریں
5. RViz2 اور logs کے ساتھ navigation مسائل debug کریں

---

## ضروریات

- باب 3.1 اور 3.2 مکمل کیے ہوں
- Navigation تصورات کی سمجھ (SLAM، costmaps، planners)
- Nav2 (ROS 2 navigation stack) سے واقفیت

---

## تعارف

**GPU سے تیز شدہ navigation اور manipulation** موبائل روبوٹس کے لیے real-time فیصلہ سازی کو قابل بناتے ہیں۔ یہ باب Isaac کے Visual SLAM اور cuMotion کو ROS 2 کے Nav2 stack کے ساتھ ضم کرنا کور کرتا ہے، پیچیدہ environments کے لیے <100ms planning latency حاصل کرتے ہوئے۔

**GPU Acceleration کیوں اہم ہے**:
- **Visual SLAM**: 30-60 FPS localization (CPU پر 5-10 FPS کے مقابلے میں)
- **Motion Planning**: 10-50ms trajectory generation (CPU پر 200-500ms کے مقابلے میں)
- **Sensor Fusion**: متعدد depth cameras کی real-time processing
- **Scalability**: SLAM + planning + perception بیک وقت چلائیں

---

## اہم اصطلاحات

:::info Glossary Terms
- **Nav2**: موبائل روبوٹس کے لیے ROS 2 navigation framework
- **Costmap**: رکاوٹوں کی موجودگی اور سفر کی قابلیت کی نمائندگی کرنے والا 2D grid
- **cuMotion**: NVIDIA کی GPU سے تیز شدہ motion planning library
- **Visual SLAM**: کیمروں کا استعمال کرتے ہوئے بیک وقت localization اور mapping
- **AMCL**: Adaptive Monte Carlo Localization (particle filter)
- **DWA**: Dynamic Window Approach (local planner)
:::

---

## بنیادی تصورات

### 1. Isaac Visual SLAM

**Isaac Visual SLAM** (vSLAM) GPU سے تیز شدہ stereo SLAM فراہم کرتا ہے 30+ FPS پر۔

**کلیدی فوائد**:
- Real-time performance بغیر dedicated LIDAR کے
- Loop closure detection برائے drift correction
- Dense mapping برائے obstacle avoidance
- Multi-camera support برائے 360° coverage

### 2. Nav2 Integration

**Nav2** ROS 2 navigation stack ہے۔ Isaac SLAM معیاری topics کے ذریعے ضم ہوتا ہے:
- \`/odom\` - Odometry data
- \`/map\` - Occupancy grid
- \`/scan\` - Laser scan (virtual من point cloud)

### 3. cuMotion: GPU Motion Planning

**cuMotion** GPU parallel computing کا استعمال کرتے ہوئے trajectory optimization کو تیز کرتا ہے۔

**روایتی Motion Planning** (CPU پر MoveIt2):
- Planning وقت: پیچیدہ scenes کے لیے 200-500ms
- Single-threaded optimization
- Limited obstacle checking

**cuMotion** (GPU سے تیز شدہ):
- Planning وقت: 10-50ms (10× تیز)
- Parallel trajectory evaluation
- Real-time رکاوٹوں سے بچنا

---

## عملی مثال: Warehouse Robot Navigation

**منظر**: TurtleBot3 shelves کو scan کرنے کے لیے خودمختار طور پر warehouse میں navigate کرتا ہے۔

**Setup**:
\`\`\`bash
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py
ros2 launch nav2_bringup navigation_launch.py
\`\`\`

**متوقع کارکردگی**:
- Map coverage: warehouse کا 95%+
- اوسط رفتار: 0.4 m/s
- رکاوٹ سے بچنا: <50cm clearance
- Mission تکمیل: 7 waypoints کے لیے 8-10 منٹ

---

## خلاصہ

اس باب نے **Isaac ROS Navigation اور Manipulation** کو کور کیا:

1. **Isaac Visual SLAM**: GPU acceleration کے ساتھ 30+ FPS stereo SLAM
2. **Nav2 Integration**: ROS 2 navigation stack کے ساتھ بے عیب رابطہ
3. **cuMotion**: GPU کے ساتھ 10× تیز motion planning
4. **خودمختار Navigation**: رکاوٹوں سے بچنے کے ساتھ waypoint missions

**اہم نکات**:
- GPU acceleration بیک وقت real-time SLAM + planning کو قابل بناتی ہے
- Isaac vSLAM معیاری \`/odom\` اور \`/map\` topics کے ذریعے Nav2 کے ساتھ ضم ہوتا ہے
- cuMotion planning کا وقت 500ms سے 35ms تک کم کرتا ہے

**اگلا باب**: reinforcement learning اور sim-to-real transfer کے لیے Isaac Gym۔`,
};

// ========================================
// PERSONALIZATION TIPS (Task 6 - Bulldog AI)
// ========================================

export const PERSONALIZATION_TIPS: Record<string, any> = {
  // For Beginner Software + None Hardware
  'beginner-none': {
    prefix: `## ✨ Personalized for Beginners

> **For Your Background**: This content has been adapted for learners new to both software and hardware.
> We'll explain concepts step-by-step with plenty of examples.

`,
    suffix: `

---

### 🎯 Learning Tips for Beginners
- Start with the basics - don't skip prerequisites
- Practice code examples in a safe environment (simulation first)
- Use online resources and communities (ROS Discourse, Stack Overflow)
- Hardware safety: Always power off before wiring, check voltage levels
- Break complex problems into smaller steps
`,
  },

  // For Intermediate Software + Basic Hardware
  'intermediate-basic': {
    prefix: `## ✨ Personalized for Intermediate Developers

> **For Your Background**: This content assumes you're comfortable with programming and have some hardware tinkering experience.
> We'll focus on ROS-specific patterns and robotics best practices.

`,
    suffix: `

---

### 🎯 Tips for Intermediate Developers
- Focus on ROS 2 architectural patterns (pub/sub, services, actions)
- Pay attention to sensor calibration and noise filtering
- Learn debugging tools: \`ros2 topic echo\`, \`rqt_graph\`, RViz
- Hardware integration: Check datasheets for I2C/SPI protocols
- Optimize for real-time performance (avoid blocking calls)
`,
  },

  // For Expert Software + Advanced Hardware
  'expert-advanced': {
    prefix: `## ✨ Personalized for Expert Engineers

> **For Your Background**: This content is optimized for experienced roboticists.
> We'll cover advanced topics, optimizations, and production considerations.

`,
    suffix: `

---

### 🎯 Advanced Engineering Notes
- **Performance**: Profile with \`ros2 run\` + \`--enable-rosout-logs\`, use DDS QoS tuning
- **Hardware**: Design for EMI/EMC compliance, thermal management, power budgeting
- **Reliability**: Implement watchdogs, graceful degradation, state machine error handling
- **Deployment**: Containerize with Docker, CI/CD for testing, OTA updates
- **Scalability**: Multi-robot coordination, cloud integration, edge computing trade-offs
`,
  },
};

/**
 * Get personalization content based on user profile
 */
export function getPersonalizedContent(
  originalContent: string,
  softwareBackground: string,
  hardwareExperience: string
): string {
  // Generate profile key
  const profileKey = `${softwareBackground.toLowerCase()}-${hardwareExperience.toLowerCase()}`;

  // Get matching tips or use intermediate as default
  const tips = PERSONALIZATION_TIPS[profileKey] || PERSONALIZATION_TIPS['intermediate-basic'];

  // Combine prefix + original + suffix
  return tips.prefix + originalContent + tips.suffix;
}

/**
 * Get Urdu translation for a chapter
 */
export function getUrduTranslation(chapterPath: string): string | null {
  // Normalize path (remove leading slash if present)
  const normalizedPath = chapterPath.startsWith('/') ? chapterPath : '/' + chapterPath;

  return URDU_TRANSLATIONS[normalizedPath] || null;
}

/**
 * Check if Urdu translation exists for a chapter
 */
export function hasUrduTranslation(chapterPath: string): boolean {
  const normalizedPath = chapterPath.startsWith('/') ? chapterPath : '/' + chapterPath;
  return normalizedPath in URDU_TRANSLATIONS;
}
