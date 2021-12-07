#include "igl/opengl/glfw/renderer.h"

#include <GLFW/glfw3.h>
#include <igl/unproject_onto_mesh.h>
#include "igl/look_at.h"
//#include <Eigen/Dense>

// OUR imports
#include <igl/AABB.h>

Renderer::Renderer() : selected_core_index(0),
next_core_id(2)
{
	core_list.emplace_back(igl::opengl::ViewerCore());
	core_list.front().id = 1;
	// C-style callbacks
	callback_init = nullptr;
	callback_pre_draw = nullptr;
	callback_post_draw = nullptr;
	callback_mouse_down = nullptr;
	callback_mouse_up = nullptr;
	callback_mouse_move = nullptr;
	callback_mouse_scroll = nullptr;
	callback_key_down = nullptr;
	callback_key_up = nullptr;

	callback_init_data = nullptr;
	callback_pre_draw_data = nullptr;
	callback_post_draw_data = nullptr;
	callback_mouse_down_data = nullptr;
	callback_mouse_up_data = nullptr;
	callback_mouse_move_data = nullptr;
	callback_mouse_scroll_data = nullptr;
	callback_key_down_data = nullptr;
	callback_key_up_data = nullptr;
	//highdpi = 1;

	xold = 0;
	yold = 0;
}

IGL_INLINE void Renderer::draw( GLFWwindow* window)
{
	using namespace std;
	using namespace Eigen;

	int width, height;
	glfwGetFramebufferSize(window, &width, &height);

	int width_window, height_window;
	glfwGetWindowSize(window, &width_window, &height_window);
	
	auto highdpi_tmp = (width_window == 0 || width == 0) ? highdpi : (width / width_window);

	if (fabs(highdpi_tmp - highdpi) > 1e-8)
	{
		post_resize(window,width, height);
		highdpi = highdpi_tmp;
	}

	for (auto& core : core_list)
	{
		core.clear_framebuffers();
	}
	int coreIndx = 1;
	if (menu)
	{
		menu->pre_draw();
		menu->callback_draw_viewer_menu();
	}
	for (auto& core : core_list)
	{
		int indx = 0;
		for (auto& mesh : scn->data_list)
		{
			
			if (mesh.is_visible & core.id)
			{// for kinematic chain change scn->MakeTrans to parent matrix
				
				core.draw(scn->MakeTransScale()*scn->CalcParentsTrans(indx).cast<float>(),mesh);
			}
			indx++;
		}

		
	}
	if (menu)
	{
		menu->post_draw();
		
	}

}

void Renderer::SetScene(igl::opengl::glfw::Viewer* viewer)
{
	scn = viewer;
}

IGL_INLINE void Renderer::init(igl::opengl::glfw::Viewer* viewer,int coresNum, igl::opengl::glfw::imgui::ImGuiMenu* _menu)
{
	scn = viewer;
	doubleVariable = 0;
	core().init(); 
	menu = _menu;
	core().align_camera_center(scn->data().V, scn->data().F);

	if (coresNum > 1)
	{	
		int width = core().viewport[2];
		int height = core().viewport[3];
		
		core().viewport = Eigen::Vector4f(0, 0, width/4, height);
		left_view = core_list[0].id;
		right_view = append_core(Eigen::Vector4f(width/4, 0, width*3/4, height));
		core_index(right_view - 1);
		for (size_t i = 0; i < scn->data_list.size(); i++)
		{
			core().toggle(scn->data_list[i].show_faces);
			core().toggle(scn->data_list[i].show_lines);
			core().toggle(scn->data_list[i].show_texture );
		}
		//Eigen::Vector3d v = -scn->GetCameraPosition();
		//TranslateCamera(v.cast<float>());

		core_index(left_view - 1);
	}

	if (menu)
	{
		menu->callback_draw_viewer_menu = [&]()
		{
			// Draw parent menu content
			menu->draw_viewer_menu(scn,core_list);


		};
	}
}

void Renderer::UpdatePosition(double xpos, double ypos)
{
	xrel = xold - xpos;
	yrel = yold - ypos;
	xold = xpos;
	yold = ypos;
}

void Renderer::MouseProcessing(int button)
{
	
	if (scn->isPicked )
	{
		if (button == 1)
		{
			float near = core().camera_dnear, far = core().camera_dfar, angle = core().camera_view_angle;
			float z = far + depth * (near - far);
			
			Eigen::Matrix4f tmpM = core().proj;
			double xToMove = -(double)xrel / core().viewport[3] * (z+2*near) * (far) / (far + 2*near) * 2.0 * tanf(angle / 360 * M_PI) / (core().camera_zoom * core().camera_base_zoom);
			double yToMove = (double)yrel / core().viewport[3] *(z+2*near) * (far ) / (far+ 2*near) * 2.0 * tanf(angle / 360 * M_PI) / (core().camera_zoom * core().camera_base_zoom);
		
			scn->data().MyTranslate( Eigen::Vector3d(xToMove, 0, 0), true);
			scn->data().MyTranslate( Eigen::Vector3d(0, yToMove, 0), true);
			scn->WhenTranslate();
		}
		else
		{
			scn->data().MyRotate(Eigen::Vector3d(1, 0, 0), (-yrel / 180));
			scn->data().MyRotate(Eigen::Vector3d(0, 1, 0), (-xrel / 180));

			// scn->data().MyRotate(scn->data().MakeTransd().block<3,3>(0,0) );

			// scn->data().MyRotate(scn->data().MakeTransd().block<3, 3>(0, 0));

		}
	}
	else
	{
		if (button == 1)
		{
			float near = core().camera_dnear, far = core().camera_dfar, angle = core().camera_view_angle;
			float z = far + 0.5f * (near - far);
			

			double xToMove = -(double)xrel / core().viewport[3] * far / z * near * 2.0f * tanf(angle / 360 * M_PI) / (core().camera_zoom * core().camera_base_zoom);
			double yToMove = (double)yrel / core().viewport[3] * far / z * near * 2.0f * tanf(angle / 360 * M_PI) / (core().camera_zoom * core().camera_base_zoom);
			scn->MyTranslate(Eigen::Vector3d(xToMove, 0, 0), true);
			scn->MyTranslate(Eigen::Vector3d(0, yToMove, 0), true);

		}
		else
		{
			scn->MyRotate(Eigen::Vector3d(1, 0, 0), (-yrel / 180));
			scn->MyRotate(Eigen::Vector3d(0, 1, 0), (-xrel / 180));

			// scn->MyRotate(scn->MakeTransd().block<3, 3>(0, 0));
			// scn->MyRotate(scn->MakeTransd().block<3, 3>(0, 0));

		}
	}
}

void Renderer::TranslateCamera(Eigen::Vector3f amt)
{
	core().camera_translation += amt;
}

void Renderer::RotateCamera(float amtX, float amtY)
{
	core().camera_eye = core().camera_eye + Eigen::Vector3f(0,amtY,0);
	Eigen::Matrix3f Mat;
		Mat << cos(amtY),0,sin(amtY),  0, 1, 0 ,  -sin(amtY), 0, cos(amtY) ;
	core().camera_eye = Mat* core().camera_eye;	
}


bool Renderer::RecMove(igl::AABB<Eigen::MatrixXd, 3>* tree, Eigen::Vector3d direction) {
	if (tree->is_leaf()) {
		tree->m_box.translate(direction);
		return true;
	}
	else {
		tree->m_box.translate(direction);
		RecMove(tree->m_right, direction);
		RecMove(tree->m_left, direction);
	}
	return false;
}

void Renderer::Move() {
	double velocity = 0.015;
	for (auto& data : scn->data_list)
	{
		if (!data.pause) {
			switch (data.direction)
			{
			case 1: // up
				data.MyTranslate(Eigen::Vector3d(0, velocity, 0), true);
				//data.tree->m_box.translate(Eigen::Vector3d(0, velocity, 0));
				data.center_dif += Eigen::Vector3d(0, velocity, 0);
				RecMove(data.tree, Eigen::Vector3d(0, velocity, 0));
				break;
			case 2: // down
				data.MyTranslate(Eigen::Vector3d(0, -velocity, 0), true);
				//data.tree->m_box.translate(Eigen::Vector3d(0, -velocity, 0));
				data.center_dif += Eigen::Vector3d(0, -velocity, 0);
				RecMove(data.tree, Eigen::Vector3d(0, -velocity, 0));
				break;
			case 3: // left
				data.MyTranslate(Eigen::Vector3d(-velocity, 0, 0), true);
				//data.tree->m_box.translate(Eigen::Vector3d(-velocity, 0, 0));
				data.center_dif += Eigen::Vector3d(-velocity, 0, 0);
				RecMove(data.tree, Eigen::Vector3d(-velocity, 0, 0));
				break;
			case 4: // right
				data.MyTranslate(Eigen::Vector3d(velocity, 0, 0), true);
				//data.tree->m_box.translate(Eigen::Vector3d(velocity, 0, 0));
				data.center_dif += Eigen::Vector3d(velocity, 0, 0);
				RecMove(data.tree, Eigen::Vector3d(velocity, 0, 0));
				break;
			case 5: //inward
				data.MyTranslate(Eigen::Vector3d(0, 0, velocity), true);
				//data.tree->m_box.translate(Eigen::Vector3d(0, 0, velocity));
				data.center_dif += Eigen::Vector3d(0, 0, velocity);
				RecMove(data.tree, Eigen::Vector3d(0, 0, velocity));
				break;
			case 6: //outward
				data.MyTranslate(Eigen::Vector3d(0, 0, -velocity), true);
				//data.tree->m_box.translate(Eigen::Vector3d(0, 0, -velocity));
				data.center_dif += Eigen::Vector3d(0, 0, -velocity);
				RecMove(data.tree, Eigen::Vector3d(0, 0, velocity));
				break;
			default:
				break;
			}
		}
	}
}

double Renderer::sign(int i, int j) {
	if ((i == 0 && j == 1) || (i == 1 && j == 2) || (i == 2 && j == 0)) {
		return 1.0;
	}
	else {
		return -1.0;
	}
}

double Renderer::c_j(Eigen::RowVector3d Ai0, Eigen::RowVector3d Ai1, Eigen::RowVector3d Bj, double sign) {
	return sign * Ai0.dot(Ai1.cross(Bj));
}

double Renderer::c_i(Eigen::RowVector3d Bj0, Eigen::RowVector3d Ai, Eigen::RowVector3d Bj1, double sign) {
	return sign * Bj0.dot(Ai.cross(Bj1));
}

bool Renderer::does_intersect(Eigen::AlignedBox<double, 3> box1, Eigen::AlignedBox<double, 3> box2, Eigen::Matrix3d rotation1, Eigen::Matrix3d rotation2) {
	Eigen::RowVector3d D = box1.center() - box2.center();
	Eigen::Matrix3d C = rotation1.transpose() * rotation2;

	// box 1 axis
	Eigen::RowVector3d A0 = rotation1.row(0);
	Eigen::RowVector3d A1 = rotation1.row(1);
	Eigen::RowVector3d A2 = rotation1.row(2);

	// box 1 extents
	Eigen::RowVector3d sizes = box1.sizes();
	double a0 = sizes(0);
	double a1 = sizes(1);
	double a2 = sizes(2);

	// box 2 axis
	Eigen::RowVector3d B0 = rotation2.row(0);
	Eigen::RowVector3d B1 = rotation2.row(1);
	Eigen::RowVector3d B2 = rotation2.row(2);
	
	// box 2 extents
	sizes = box2.sizes();
	double b0 = sizes(0);
	double b1 = sizes(1);
	double b2 = sizes(2);

	// checking intersection
	// NOT SURE ABOUT THE CALCULATION OF cij
	double c00 = c_i(B1, A0, B2, sign(1, 2));
	double c01 = c_i(B0, A0, B2, sign(0, 2));
	double c02 = c_i(B0, A0, B1, sign(0, 1));

	// 1st check
	if (std::abs(A0.dot(D)) > a0 + (b0 * std::abs(c00) + b1 * std::abs(c01) + b2 * std::abs(c02))) {
		return false;
	}

	double c10 = c_i(B1, A1, B2, sign(1, 2));
	double c11 = c_i(B0, A1, B2, sign(0, 2));
	double c12 = c_i(B0, A1, B1, sign(0, 1));

	// 2nd check
	if (std::abs(A1.dot(D)) > a1 + (b0 * std::abs(c10) + b1 * std::abs(c11) + b2 * std::abs(c12))) {
		return false;
	}

	double c20 = c_i(B1, A1, B2, sign(1, 2));
	double c21 = c_i(B0, A1, B2, sign(0, 2));
	double c22 = c_i(B0, A1, B1, sign(0, 1));

	// 3rd check
	if (std::abs(A2.dot(D)) > a2 + (b0 * std::abs(c20) + b1 * std::abs(c21) + b2 * std::abs(c22))) {
		return false;
	}

	// 4th check
	if (std::abs(B0.dot(D)) > b0 + (a0 * std::abs(c00) + a1 * std::abs(c10) + a2 * std::abs(c20))) {
		return false;
	}

	// 5th check
	if (std::abs(B1.dot(D)) > b1 + (a0 * std::abs(c01) + a1 * std::abs(c11) + a2 * std::abs(c21))) {
		return false;
	}

	// 6th check
	if (std::abs(B2.dot(D)) > b2 + (a0 * std::abs(c02) + a1 * std::abs(c12) + a2 * std::abs(c22))) {
		return false;
	}

	// 7th check
	if (std::abs(c10 * A2.dot(D) - c20 * A1.dot(D)) > a1 * std::abs(c20) + a2 * std::abs(c10) + b1 * std::abs(c02) + b2 * std::abs(c01)) {
		return false;
	}

	// 8th check
	if (std::abs(c11 * A2.dot(D) - c21 * A1.dot(D)) > a1 * std::abs(c21) + a2 * std::abs(c11) + b0 * std::abs(c02) + b2 * std::abs(c00)) {
		return false;
	}

	// 9th eck
	if (std::abs(c12 * A2.dot(D) - c22 * A1.dot(D)) > a1 * std::abs(c22) + a2 * std::abs(c12) + b0 * std::abs(c01) + b1 * std::abs(c00)) {
		return false;
	}

	// 10th eck
	if (std::abs(c20 * A0.dot(D) - c00 * A2.dot(D)) > a0 * std::abs(c20) + a2 * std::abs(c00) + b1 * std::abs(c12) + b2 * std::abs(c11)) {
		return false;
	}

	// 11th eck
	if (std::abs(c21 * A0.dot(D) - c01 * A2.dot(D)) > a0 * std::abs(c21) + a2 * std::abs(c01) + b0 * std::abs(c12) + b2 * std::abs(c10)) {
		return false;
	}

	// 12th eck
	if (std::abs(c22 * A0.dot(D) - c02 * A2.dot(D)) > a0 * std::abs(c22) + a2 * std::abs(c02) + b0 * std::abs(c11) + b1 * std::abs(c10)) {
		return false;
	}

	// 13th eck
	if (std::abs(c00 * A1.dot(D) - c10 * A0.dot(D)) > a0 * std::abs(c10) + a1 * std::abs(c00) + b1 * std::abs(c22) + b2 * std::abs(c21)) {
		return false;
	}

	// 14th eck
	if (std::abs(c01 * A1.dot(D) - c11 * A0.dot(D)) > a0 * std::abs(c11) + a1 * std::abs(c01) + b0 * std::abs(c22) + b2 * std::abs(c20)) {
		return false;
	}

	// 14th eck
	if (std::abs(c02 * A1.dot(D) - c12 * A0.dot(D)) > a0 * std::abs(c12) + a1 * std::abs(c02) + b0 * std::abs(c21) + b1 * std::abs(c20)) {
		return false;
	}

	return true;
}


bool Renderer::CheckCollisionRec(igl::opengl::ViewerData obj1, igl::opengl::ViewerData obj2, igl::AABB<Eigen::MatrixXd, 3>* tree1, igl::AABB<Eigen::MatrixXd, 3>* tree2){

	// translating the boxes along the tree
	/*if (d != 0) {
		tree1->m_box.translate(obj1.center_dif);
		tree2->m_box.translate(obj2.center_dif);
	}*/

	if(tree1->is_leaf() && tree2->is_leaf()){	
		if (does_intersect(tree1->m_box, tree2->m_box, obj1.GetRotation(), obj2.GetRotation())) {
			std::cout << "fuck " << std::endl;
			std::cout << tree1->m_box.center() << std::endl;
			std::cout << tree2->m_box.center() << std::endl;
			obj1.draw_box(tree1->m_box);
			obj2.draw_box(tree2->m_box);

			return true;
		}

		return false;
	}
	// does_intersect(tree1->m_box, tree2->m_box, obj1.GetRotation(), obj2.GetRotation())
	if(does_intersect(tree1->m_box, tree2->m_box, obj1.GetRotation(), obj2.GetRotation())){
		 return CheckCollisionRec(obj1, obj2, tree1->is_leaf() ? tree1 : tree1->m_right, tree2->is_leaf() ? tree2 : tree2->m_right)
		 	|| CheckCollisionRec(obj1, obj2, tree1->is_leaf() ? tree1 : tree1->m_right, tree2->is_leaf() ? tree2 : tree2->m_left)
		 	|| CheckCollisionRec(obj1, obj2, tree1->is_leaf() ? tree1 : tree1->m_left, tree2->is_leaf() ? tree2 : tree2->m_right)
		 	|| CheckCollisionRec(obj1, obj2, tree1->is_leaf() ? tree1 : tree1->m_left, tree2->is_leaf() ? tree2 : tree2->m_left);

		 //std::cout << d << std::endl;
		/*++d;
		stam |= CheckCollisionRec(obj1, obj2, tree1->is_leaf() ? tree1 : tree1->m_right, tree2->is_leaf() ? tree2 : tree2->m_right, d);
		stam |= CheckCollisionRec(obj1, obj2, tree1->is_leaf() ? tree1 : tree1->m_right, tree2->is_leaf() ? tree2 : tree2->m_left, d);
		stam |= CheckCollisionRec(obj1, obj2, tree1->is_leaf() ? tree1 : tree1->m_left, tree2->is_leaf() ? tree2 : tree2->m_right, d);
		stam |= CheckCollisionRec(obj1, obj2, tree1->is_leaf() ? tree1 : tree1->m_left, tree2->is_leaf() ? tree2 : tree2->m_left, d);*/
	}

	return false;
}

bool Renderer::CheckCollision() {
	/*if(CheckCollisionRec(scn->data_list[0].tree, scn->data_list[1].tree)){
		scn->data_list[0].Pause();
		scn->data_list[1].Pause();
		return true;
	}*/


	for (auto& object1 : scn->data_list)
	{
		for (auto& object2 : scn->data_list)
		{
			if (object1.id != object2.id) { // checking we're not checking the collision between an object and itself
				if (!object1.pause || !object2.pause) {
					if (CheckCollisionRec(object1, object2, object1.tree, object2.tree)) {
						//std::cout << "fuck "<< std::endl;
						object1.center_dif = Eigen::Vector3d(0, 0, 0);
						object2.center_dif = Eigen::Vector3d(0, 0, 0);
						object1.pause = true;
						object2.pause = true;
						return true;
					}
				}
			}
		}
	}
	return false;
}



Renderer::~Renderer()
{
	//if (scn)
	//	delete scn;
}

double Renderer::Picking(double newx, double newy)
{
		int fid;
		//Eigen::MatrixXd C = Eigen::MatrixXd::Constant(scn->data().F.rows(), 3, 1);
		Eigen::Vector3f bc;
		double x = newx;
		double y = core().viewport(3) - newy;
		
		Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
		
		igl::look_at(core().camera_eye, core().camera_center, core().camera_up, view);
		//std::cout << "view matrix\n" << view << std::endl;
		view = view * (core().trackball_angle * Eigen::Scaling(core().camera_zoom * core().camera_base_zoom)
				* Eigen::Translation3f(core().camera_translation + core().camera_base_translation)).matrix() * scn->MakeTransScale()* scn->CalcParentsTrans(scn->selected_data_index).cast<float>() * scn->data().MakeTransScale();
		bool picked = igl::unproject_onto_mesh(Eigen::Vector2f(x, y), view,
			core().proj, core().viewport, scn->data().V, scn->data().F, fid, bc);
		scn->isPicked = scn->isPicked | picked;
		if (picked)
		{
			Eigen::Vector3i face = scn->data().F.row(fid);
			Eigen::Matrix3d vertices ;
			Eigen::Vector4f p,pp ;

			vertices.col(0) = scn->data().V.row(face(0));
			vertices.col(1) =  scn->data().V.row(face(1));
			vertices.col(2) = scn->data().V.row(face(2));
		
			p <<  vertices.cast<float>() * bc ,1;
			p = view * p;
			//std::cout << scn->data().V.row(face(0)) << std::endl;
			pp = core().proj * p;
			//glReadPixels(x,  y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
			z = pp(2);
			return p(2);
			
		}
		return 0;
}

IGL_INLINE void Renderer::resize(GLFWwindow* window,int w, int h)
	{
		if (window) {
			glfwSetWindowSize(window, w / highdpi, h / highdpi);
		}
		post_resize(window,w, h);
	}

	IGL_INLINE void Renderer::post_resize(GLFWwindow* window, int w, int h)
	{
		if (core_list.size() == 1)
		{
	
			core().viewport = Eigen::Vector4f(0, 0, w, h);
		}
		else
		{
			// It is up to the user to define the behavior of the post_resize() function
			// when there are multiple viewports (through the `callback_post_resize` callback)
			core(left_view).viewport = Eigen::Vector4f(0, 0, w / 4, h);
			core(right_view).viewport = Eigen::Vector4f(w / 4, 0, w - (w / 4), h);

		}
		//for (unsigned int i = 0; i < plugins.size(); ++i)
		//{
		//	plugins[i]->post_resize(w, h);
		//}
		if (callback_post_resize)
		{
			callback_post_resize(window, w, h);
		}
	}

	IGL_INLINE igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/)
	{
		assert(!core_list.empty() && "core_list should never be empty");
		int core_index;
		if (core_id == 0)
			core_index = selected_core_index;
		else
			core_index = this->core_index(core_id);
		assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
		return core_list[core_index];
	}

	IGL_INLINE const igl::opengl::ViewerCore& Renderer::core(unsigned core_id /*= 0*/) const
	{
		assert(!core_list.empty() && "core_list should never be empty");
		int core_index;
		if (core_id == 0)
			core_index = selected_core_index;
		else
			core_index = this->core_index(core_id);
		assert((core_index >= 0 && core_index < core_list.size()) && "selected_core_index should be in bounds");
		return core_list[core_index];
	}

	IGL_INLINE bool Renderer::erase_core(const size_t index)
	{
		assert((index >= 0 && index < core_list.size()) && "index should be in bounds");
		//assert(data_list.size() >= 1);
		if (core_list.size() == 1)
		{
			// Cannot remove last viewport
			return false;
		}
		core_list[index].shut(); // does nothing
		core_list.erase(core_list.begin() + index);
		if (selected_core_index >= index && selected_core_index > 0)
		{
			selected_core_index--;
		}
		return true;
	}

	IGL_INLINE size_t Renderer::core_index(const int id) const {
		for (size_t i = 0; i < core_list.size(); ++i)
		{
			if (core_list[i].id == id)
				return i;
		}
		return 0;
	}

	IGL_INLINE int Renderer::append_core(Eigen::Vector4f viewport, bool append_empty /*= false*/)
	{
		core_list.push_back(core()); // copies the previous active core and only changes the viewport
		core_list.back().viewport = viewport;
		core_list.back().id = next_core_id;
		next_core_id <<= 1;
		if (!append_empty)
		{
			for (auto& data : scn->data_list)
			{
				data.set_visible(true, core_list.back().id);
				//data.copy_options(core(), core_list.back());
			}
		}
		selected_core_index = core_list.size() - 1;
		return core_list.back().id;
	}

	//IGL_INLINE void Viewer::select_hovered_core()
	//{
	//	int width_window, height_window = 800;
	//   glfwGetFramebufferSize(window, &width_window, &height_window);
	//	for (int i = 0; i < core_list.size(); i++)
	//	{
	//		Eigen::Vector4f viewport = core_list[i].viewport;

	//		if ((current_mouse_x > viewport[0]) &&
	//			(current_mouse_x < viewport[0] + viewport[2]) &&
	//			((height_window - current_mouse_y) > viewport[1]) &&
	//			((height_window - current_mouse_y) < viewport[1] + viewport[3]))
	//		{
	//			selected_core_index = i;
	//			break;
	//		}
	//	}
	//}